// EHC2025 - Electric Heater Controller 2025 - ESP32 based

/*
  (Source: Google)
	To calculate the shower water outlet temperature,
	you need to consider the heater power,
	the water flow rate, and the initial water temperature.
	The formula for calculating the temperature change is
	ΔT = (Power * Time) / (Mass * Specific Heat).
	However, since the flow rate is given in liters per minute,
	we need to convert to units compatible with the
	power and specific heat of the water.

	Calculation steps:

	1. Convert flow rate to compatible units:
	2 liters per minute is equivalent to 2/60 = 0.0333 liters per second.
	The density of water is approximately 1 kg/liter, so 0.0333 liters/s = 0.0333 kg/s.
	The specific heat of water is approximately 4186 J/(kg°C).

	2. Calculate the heat added to the water:
	The shower's power is 5400 watts, which translates to 5400 joules per second.
	To calculate the temperature change in 1 second, we use: ΔT = (Power * Time) / (Mass * Specific Heat).
	ΔT = (5400 W * 1 s) / (0.0333 kg * 4186 J/(kg °C))
	ΔT ≈ 38.9 °C

	3. Calculate the final water temperature:
	The initial water temperature is 25°C.
	The calculated temperature change is approximately 38.9°C.
	The final water temperature will be: 25°C + 38.9°C = 63.9°C
 */


// NTC B3950 Thermistor
// the formula for temp in kelvin is
//                 1
// T = ----------------------------
//     1/To + (1/beta) * ln(Rt/Ro)
//
// https://en.wikipedia.org/wiki/Thermistor

// ESP32 ADC non-liear issue
// https://www.esp32.com/viewtopic.php?t=1045
// https://github.com/espressif/esp-idf/issues/164
// https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/adc.html#adc-calibration
// https://github.com/e-tinkers/ntc-thermistor-with-arduino-and-esp32/blob/master/ntc_3950.ino

#include "BluetoothSerial.h"
#include "TM1637Display.h"
#include "VirtuinoCM.h" // https://github.com/iliaslamprou/virtuino

// Module connection pins (Digital Pins)
#define CLK 26
#define DIO 27

#define BTN_P_IN_pin 14
#define BTN_S_IN_pin 12
#define BTN_M_IN_pin 13

#define NTC_COLD_pin 36
#define NTC_HOT_pin 39

#define TRIAC_pin 4  // BTA41
#define ZCD_pin 15   // PC817

#define FLOW_COLD_pin 32  // YF-S201 sensor 1
#define FLOW_HOT_pin 33   // YF-S201 sensor 2

#define DIMMER_ON_TIMER_MIN 800  // 8000us
#define DIMMER_ON_TIMER_MAX 50   // 500us

#define DIMM_MAP_MIN 515
#define DIMM_MAP_MAX 37

const int DimmMap_array[] = {
  515, 445, 426, 413, 402, 392, 384, 377, 370, 363,
  357, 352, 346, 341, 336, 332, 327, 323, 318, 314,
  310, 306, 302, 298, 295, 291, 287, 284, 280, 277,
  274, 270, 267, 264, 261, 257, 254, 251, 248, 245,
  242, 239, 236, 233, 230, 227, 224, 221, 218, 216,
  213, 210, 207, 204, 201, 199, 196, 193, 190, 187,
  184, 182, 179, 176, 173, 170, 168, 165, 162, 159,
  156, 153, 150, 147, 144, 141, 138, 135, 132, 129,
  126, 123, 120, 117, 113, 110, 107, 103, 100, 96,
  92, 89, 85, 81, 76, 72, 67, 62, 56, 49,
  37
};

#define TEMPERATURE_MAX 42.5      // °c
#define TEMPERATURE_LIMIT 47.5    // °c
#define TEMPERATURE_RELEASE 40.0  // °c
#define TEMPERATURE_MIN -9        // °c

#define EEPROM_SIZE 16

#define MENU_RUN 0
#define MENU_P1 1
#define MENU_P2 2
#define MENU_P3 3
#define MENU_P4 4
#define MENU_P5 5
#define MENU_P6 6
#define MENU_P7 7
#define MENU_P8 8
#define MENU_P9 9
#define MENU_P10 10
#define MENU_P11 11
#define MENU_P12 12
#define MENU_P13 13
#define MENU_P14 14
#define MENU_P_MAX 14

unsigned int heater_power_w = 5400;  // Watts

// The ESP32 lookup table varies from device to device, use the program from
// https://github.com/e-tinkers/esp32-adc-calibrate
// to generate lookup table for your own ESP32
extern const float ADC_LUT[4096];  // located on file: ADC_LUT.ino

double adcMax = 4095.0;  // ADC resolution 12-bit (0-4095)
double Vs = 3.3;         // supply voltage
double R1 = 5000.0;      // voltage divider resistor value
double Beta = 3950.0;    // Beta value
double To = 298.15;      // Temperature in Kelvin for 25 degree Celsius
double Ro = 10000.0;     // Resistance of Thermistor at 25 degree Celsius

uint32_t millis1 = 0;
uint32_t millis2 = 0;

volatile uint16_t dimm_buff = 0;

volatile uint32_t flow_cold_us = 0;
volatile uint32_t flow_hot_us = 0;

volatile uint32_t timer_flow_cold_us = 0;
volatile uint32_t timer_flow_hot_us = 0;

volatile uint32_t timeout_flow_cold_us_x10 = 0;
volatile uint32_t timeout_flow_hot_us_x10 = 0;

hw_timer_t* timer_10us = NULL;

bool dimm_up = false;

uint8_t dimm_power = 0;

bool dimm_enable_temper_out = false;

volatile bool dimm_enable = false;

volatile uint32_t zc_timer_10us = 0;
volatile uint32_t zc_time_us = 0;

volatile uint16_t dimmer_on_timer = 0;
volatile uint8_t dimmer_off_timer = 0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE extIntMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE loopMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t power_calc_buff = 0;

float freq = 0;
float flow_cold = 0;
float rate_flow_cold = 0;
float flow_hot = 0;
float rate_flow_hot = 0;
float flow_total = 0;
float temperature_cold = 0;
float temperature_hot = 0;
uint8_t power_calc = 0;
float power_reducer = 0;

String device_name = "EHC2025";

// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;

//-------------VirtuinoCM  Library and settings --------------
VirtuinoCM virtuino;
#define V_memory_count 32  // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];   // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.

boolean debug = true;  // set this variable to false on the finale code to decrease the request time.

TM1637Display display(CLK, DIO, 5);  // 5us

const uint8_t data_all_on[] = { 0b01111111, 0b11111111, 0b11111111, 0b11111111 };
uint8_t disp_buff[]{ 0, 0, 0, 0 };

uint32_t disp_millis = 0;
bool LED_power = true;
bool LED_heater = false;
bool LED_heater_on = true;
bool LED_heater_blink = false;

bool LED_blink = false;

bool KEY_set_pressed = false;
bool KEY_set_pressed_old = false;
bool KEY_set_longpress = false;
bool KEY_set_longpress_old = false;
uint8_t KEY_set_pressed_cnt = 0;

bool KEY_plus_pressed = false;
bool KEY_plus_pressed_old = false;
bool KEY_plus_longpress = false;
bool KEY_plus_longpress_old = false;
uint8_t KEY_plus_pressed_cnt = 0;

bool KEY_minus_pressed = false;
bool KEY_minus_pressed_old = false;
bool KEY_minus_longpress = false;
bool KEY_minus_longpress_old = false;
uint8_t KEY_minus_pressed_cnt = 0;

uint32_t key_millis = 0;

uint8_t menu_state = 0;

uint8_t temperature_out = 25;

/*
	Operation:
	Minimum combined flow rate: 1 L/min
	Minimum heating ratio: 0.25 (hot/cold); 0% heating; below 0.25 is always the cold inlet temperature.
	Maximum heating ratio: 0.75 (hot/cold); 100% heating; above 0.75 is always the maximum outlet temperature.
	Maximum outlet temperature: 45°C
	Minimum outlet temperature: Cold inlet temperature.
  
	Example 1:
	Hot flow rate: 0.5 l/min
	Cold flow rate: 0.9 l/min
	Summary flow rate: 0.5 + 0.9 = 1 l/min
	Hot/cold ratio: 0.5/0.9 = 0.555555556

	Heating rate: (yRel - 0.25) / (0.75 - 0.25)
	Heating rate: (0.555555556 - 0.25) / (0.75 - 0.25)
	Heating rate: 0.305555556 / 0.50
	Heating rate: 0.611111112

	Cold inlet temperature: 25°C
	Maximum outlet temperature: 45°C

	Outlet temperature: ((Maximum outlet temperature - Cold inlet temperature) * Heating rate) + Cold inlet temperature = 27.50000004°C
	Outlet Temperature: ((45°C - 25°C) * 0.611111112) + 25°C
	Outlet Temperature: (20°C * 0.611111112) + 25°C
	Outlet Temperature: 12.22222224°C + 25°C
	Outlet Temperature: 37.22222224°C
	Outlet Temperature: 37.22222224°C < Maximum Outlet Temperature
	Outlet Temperature: 37.22222224°C

	Example 2:
	Hot flow rate: 0.8 l/min
	Cold flow rate: 0.4 l/min
	Summary flow rate: 0.8 + 0.4 = 1.2 l/min
	Hot/cold ratio: 0.8/0.4 = 2.0

	Heating rate: (yRel - 0.25) / (0.75 - 0.25)
	Heating rate: (2 - 0.25) / (0.75 - 0.25)
	Heating rate: 1.75 / 0.50
	Heating rate: 3.5

	Cold inlet temperature: 20°C
	Maximum outlet temperature: 45°C

	Outlet temperature: ((Maximum outlet temperature - Cold inlet temperature) * Heating rate) + Cold inlet temperature = 27.50000004°C
	Outlet temperature: ((45°C - 20°C) * 3.5) + 25°C
	Outlet Temperature: (25°C * 3.5) + 25°C
	Outlet Temperature: 87.5°C + 25°C
	Outlet Temperature: 112.5°C
	Outlet Temperature: 112.5°C > Maximum Outlet Temperature
	Outlet Temperature: Maximum Outlet Temperature = 45°C
	Outlet Temperature: 45°C
 */

// ΔT = (Power * Time) / (Mass * Specific Heat)
uint8_t calc_power_out(float temp_in, float temp_out_max, float water_flow_cold, float water_flow_hot) {
  float water_flow_sum = 0.0;
  float hot_cold_rate = 0.0;
  float heat_rate = 0.0;
  float temp_out = 0.0;
  const float time_s = 1;
  float mass = 0.0;
  const float cal_JkgC = 4186;  // 4186 J/(kg·°C)
  float dT = 0.0;
  float pot_w = 0.0;
  float pot_rate = 0.0;

  water_flow_sum = water_flow_cold + water_flow_hot;

  if ((water_flow_sum == 0) || (water_flow_hot == 0)) {
    return 0;
  }

  hot_cold_rate = water_flow_hot / water_flow_sum;

  // Serial.print("hot_cold_rate: ");
  // Serial.println(hot_cold_rate, 2);

  heat_rate = (hot_cold_rate - 0.25) / (0.75 - 0.25);
  // Serial.print("heat_rate: ");
  // Serial.println(heat_rate, 2);

  temp_out = ((temp_out_max - temp_in) * heat_rate) + temp_in;
  // Serial.print("temp_out: ");
  // Serial.println(temp_out, 2);

  if (temp_out > temp_out_max) {
    temp_out = temp_out_max;
  }

  // Serial.print("temp_out: ");
  // Serial.println(temp_out, 2);

  mass = (water_flow_cold + water_flow_hot) / 60.0;  // Liters per minute (flow) to kg/s (mass)

  dT = temp_out - temp_in;
  // Serial.print("dT: ");
  // Serial.println(dT, 2);

  pot_w = (mass * cal_JkgC) * dT;
  // Serial.print("pot_w: ");
  // Serial.println(pot_w, 2);

  pot_rate = (pot_w / heater_power_w) * 100;
  // Serial.print("pot_rate: ");
  // Serial.println(pot_rate, 2);

  uint8_t pot_rate_ret = pot_rate;

  if (pot_rate > 100.0) {
    pot_rate_ret = 100;
  } else if (pot_rate < 0.0) {
    pot_rate_ret = 0;
  }

  return pot_rate_ret;
}

uint16_t convert_Dimm(uint8_t power_output) {
  uint16_t result = 0;

  if (power_output <= 100) {
    uint16_t value = DimmMap_array[power_output];

    result = map(value, DIMM_MAP_MIN, DIMM_MAP_MAX, DIMMER_ON_TIMER_MIN, DIMMER_ON_TIMER_MAX);

    return result;
  } else {
    return 0;
  }
}

void ARDUINO_ISR_ATTR timer_10us_ISR() {
  // portENTER_CRITICAL_ISR(&timerMux);
  // flow meter
  if (timeout_flow_cold_us_x10 > 0) {
    timeout_flow_cold_us_x10--;
    timer_flow_cold_us++;
  } else {
    flow_cold_us = 0;
    timer_flow_cold_us = 0;
  }

  if (timeout_flow_hot_us_x10 > 0) {
    timeout_flow_hot_us_x10--;
    timer_flow_hot_us++;
  } else {
    flow_hot_us = 0;
    timer_flow_hot_us = 0;
  }

  // dimmer
  zc_timer_10us++;

  if (dimmer_on_timer > 0) {
    dimmer_on_timer--;

    if (dimmer_on_timer == 0) {
      digitalWrite(TRIAC_pin, dimm_enable);
    }
  } else {
    if (dimmer_off_timer > 0) {
      dimmer_off_timer--;
    } else {
      digitalWrite(TRIAC_pin, LOW);
    }
  }
  // portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR flow_cold_ext_ISR() {
  if (digitalRead(FLOW_COLD_pin) == LOW) {
    flow_cold_us = timer_flow_cold_us;
    timer_flow_cold_us = 0;
    timeout_flow_cold_us_x10 = 75000;  // 750ms
  }
}

void IRAM_ATTR flow_hot_ext_ISR() {
  if (digitalRead(FLOW_HOT_pin) == LOW) {
    flow_hot_us = timer_flow_hot_us;
    timer_flow_hot_us = 0;
    timeout_flow_hot_us_x10 = 75000;  // 750ms
  }
}

void IRAM_ATTR zero_cross_ext_ISR() {
  // portENTER_CRITICAL_ISR(&extIntMux);
  if (digitalRead(ZCD_pin) == LOW) {
    zc_time_us = zc_timer_10us;
    zc_timer_10us = 0;

    if ((dimm_buff <= DIMMER_ON_TIMER_MIN) && (dimm_buff >= DIMMER_ON_TIMER_MAX)) {
      dimmer_on_timer = dimm_buff;
    } else {
      dimmer_on_timer = 0;
    }

    dimmer_off_timer = 5;
  }
  // portEXIT_CRITICAL_ISR(&extIntMux);
}

void setup() {
  // put your setup code here, to run once:
  pinMode(FLOW_COLD_pin, INPUT_PULLUP);
  pinMode(FLOW_HOT_pin, INPUT_PULLUP);

  pinMode(TRIAC_pin, OUTPUT);
  pinMode(ZCD_pin, INPUT_PULLUP);

  pinMode(NTC_COLD_pin, INPUT);
  pinMode(NTC_HOT_pin, INPUT);

  digitalWrite(TRIAC_pin, LOW);

  Serial.begin(115200);  // Initialize the serial communication:

  SerialBT.begin(device_name);  //Bluetooth device name

  display.setBrightness(0x01);

  // All segments on
  display.setSegments(data_all_on, 4);
  delay(2000);

  Serial.println("Start");

  // Note: CHANGE mode resulted in better stability
  attachInterrupt(ZCD_pin, zero_cross_ext_ISR, CHANGE);  // CHANGE FALLING RISING

  // Note: CHANGE mode resulted in better stability
  attachInterrupt(FLOW_COLD_pin, flow_cold_ext_ISR, CHANGE);  // CHANGE FALLING RISING
  attachInterrupt(FLOW_HOT_pin, flow_hot_ext_ISR, CHANGE);    // CHANGE FALLING RISING

  timer_10us = timerBegin(100000);                    // <<< Set timer frequency Mhz
  timerAttachInterrupt(timer_10us, &timer_10us_ISR);  // <<< Attach Timer0_ISR function to our timer.
  // Set alarm to call onTimer function(value in microseconds x10).
  // Repeat the alarm (third parameter) with unlimited count = 0 (fourth parameter).
  timerAlarm(timer_10us, 1, true, 0);  // <<< 10us

  virtuino.begin(onReceived, onRequested, 256);  //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((millis() - millis1) >= 200) {
    millis1 = millis();

    flow_calc();

    temperature_cold = get_temperature(NTC_COLD_pin);
    temperature_hot = get_temperature(NTC_HOT_pin);

    power_handler();

    dimm_buff = convert_Dimm(power_calc_buff);  // 0-100% to dimm_us

    // Serial data (debug)
    data_print_serial();

    // Virtuino data
    data_load_virtuino();

    // Display
    display_show();
  }

  virtuinoRun();
}

void display_show() {
  LED_blink = !LED_blink;

  if (menu_state == MENU_RUN) {
    if (LED_heater_on == true) {
      LED_heater = LED_blink;
    }

    disp_set_deg(temperature_hot, LED_power, LED_heater);

    if ((temperature_hot <= -10) || (temperature_hot > TEMPERATURE_MAX)) {
      if (LED_blink == false) {
        disp_buff[0] = 0;
        disp_buff[1] = 0;
      }
    }

  } else if ((menu_state >= MENU_P1) && (menu_state <= MENU_P_MAX)) {
    disp_buff[0] = 0b01110011;  // P

    uint8_t p10 = menu_state / 10;
    uint8_t p1 = menu_state - (p10 * 10);

    if (menu_state <= 9) {
      disp_buff[1] = display.encodeDigit(menu_state);
    } else {
      disp_buff[1] = display.encodeDigit(p10);
    }

    if (menu_state <= 9) {
      disp_buff[2] = 0 | 0x80;
    } else {
      disp_buff[2] = display.encodeDigit(p1) | 0x80;
    }

    disp_buff[3] = 0;

    if (LED_blink == false) {
      disp_buff[1] = 0;
      disp_buff[2] = 0;
    }
  }

  if ((digitalRead(BTN_P_IN_pin) == true) || (digitalRead(BTN_S_IN_pin) == true) || (digitalRead(BTN_M_IN_pin) == true)) {
    disp_buff[2] &= 0b01111111;
  }

  display.setSegments(disp_buff, 4);
}

void flow_calc() {
  freq = 0;
  flow_cold = 0;
  rate_flow_cold = 0;
  flow_hot = 0;
  rate_flow_hot = 0;
  flow_total = 0;

  if (flow_cold_us != 0) {
    freq = 100000;
    freq /= flow_cold_us;
    flow_cold = freq;
    flow_cold /= 7.5;
  }

  freq = 0;

  if (flow_hot_us != 0) {
    freq = 100000;
    freq /= flow_hot_us;
    flow_hot = freq;
    flow_hot /= 7.5;
  }

  flow_total = flow_cold;
  flow_total += flow_hot;

  if (flow_total >= 0.5) {
    if (flow_cold != 0) {
      rate_flow_cold = flow_cold;
      rate_flow_cold /= flow_total;
      rate_flow_cold *= 100;
    }

    if (flow_hot != 0) {
      rate_flow_hot = flow_hot;
      rate_flow_hot /= flow_total;
      rate_flow_hot *= 100;
    }
  }
}

void power_handler() {
  power_calc = 0;

  if (flow_total > 0.5) {
    power_calc = calc_power_out(temperature_cold, TEMPERATURE_MAX, flow_cold, flow_hot);
  }

  if (((temperature_hot > TEMPERATURE_LIMIT) || (temperature_hot < TEMPERATURE_MIN) || (temperature_cold > TEMPERATURE_LIMIT) || (temperature_cold < TEMPERATURE_MIN))
      && (dimm_enable_temper_out == true)) {
    dimm_enable_temper_out = false;
  } else if ((temperature_hot < TEMPERATURE_RELEASE) && (temperature_hot > TEMPERATURE_MIN) && (temperature_cold < TEMPERATURE_RELEASE) && (temperature_cold > TEMPERATURE_MIN)
             && (dimm_enable_temper_out == false)) {
    dimm_enable_temper_out = true;
  }

  if (power_calc > 0) {
    dimm_enable = dimm_enable_temper_out;
  } else {
    dimm_enable = false;
  }

  if (dimm_enable == false) {
    power_calc = 0;
  }

  if ((temperature_hot > TEMPERATURE_MAX) && (temperature_hot < TEMPERATURE_LIMIT)) {
    power_reducer = temperature_hot;
    power_reducer -= TEMPERATURE_MAX;
    power_reducer *= 0.2;
    power_reducer = 1 - power_reducer;
    power_calc *= power_reducer;
  }

  if ((power_calc_buff > power_calc) || ((power_calc_buff + 10) > power_calc)) {
    power_calc_buff = power_calc;
  } else if (power_calc_buff != power_calc) {
    power_calc_buff += 10;
  }

  if (power_calc_buff > 100) {
    power_calc_buff = 100;
  }
}

void data_load_virtuino() {
  V[0] = temperature_cold;
  V[1] = temperature_hot;
  V[2] = flow_total * 10;
  V[3] = rate_flow_cold;
  V[4] = rate_flow_hot;
  V[5] = power_calc_buff;
}

void data_print_serial() {
  Serial.print("TC: ");
  Serial.print(temperature_cold, 2);

  Serial.print(" TH: ");
  Serial.print(temperature_hot, 2);

  Serial.print(" Tot: ");
  Serial.print(flow_total, 2);

  Serial.print(" FC: ");
  Serial.print(flow_cold, 2);

  Serial.print(" [");
  Serial.print(rate_flow_cold);

  Serial.print("] FH: ");
  Serial.print(flow_hot, 2);

  Serial.print(" [");
  Serial.print(rate_flow_hot);

  Serial.print("] PWR: ");
  Serial.print(power_calc);

  Serial.print("[BUF: ");
  Serial.print(power_calc_buff);

  Serial.print("] ZC: ");
  Serial.print(get_freq(zc_time_us));
  Serial.println("(Hz)");
}

//============================================================== onCommandReceived
//==============================================================
/* This function is called every time Virtuino app sends a request to server to change a Pin value
 * The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin 
 * The 'variableIndex' is the pin number index of Virtuino app
 * The 'valueAsText' is the value that has sent from the app   */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    float value = valueAsText.toFloat();                           // convert the value to float. The valueAsText have to be numerical
    if (variableIndex < V_memory_count) V[variableIndex] = value;  // copy the received value to arduino V memory array
  }
}

//==============================================================
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex) {
  if (variableType == 'V') {
    if (variableIndex < V_memory_count) return String(V[variableIndex]);  // return the value of the arduino V memory array
  }
  return "";
}

//============================================================== virtuinoRun
void virtuinoRun() {
  while (SerialBT.available()) {
    char tempChar = SerialBT.read();
    if (tempChar == CM_START_CHAR) {        // a new command is starting...
      virtuino.readBuffer = CM_START_CHAR;  // copy the new command to the virtuino readBuffer
      virtuino.readBuffer += SerialBT.readStringUntil(CM_END_CHAR);
      virtuino.readBuffer += CM_END_CHAR;
      if (debug) Serial.println("\nCommand= " + virtuino.readBuffer);
      String* response = virtuino.getResponse();  // get the text that has to be sent to Virtuino as reply. The library will check the inptuBuffer and it will create the response text
      if (debug) Serial.println("Response : " + *response);
      SerialBT.print(*response);
      break;
    }
  }
}

float get_temperature(int NTC_pin) {
  double Vout, Rt = 0;
  // double T, Tc, Tf = 0;
  double T, Tc = 0;

  double adc_val = 0;

  for (uint16_t i = 0; i < 2; i++) {
    adc_val += analogRead(NTC_pin);
  }

  adc_val /= 2;

  adc_val = ADC_LUT[(int)adc_val];

  Vout = adc_val * Vs / adcMax;
  Rt = R1 * Vout / (Vs - Vout);

  T = 1 / (1 / To + log(Rt / Ro) / Beta);  // Temperature in Kelvin
  Tc = T - 273.15;                         // Celsius
  //Tf = Tc * 9 / 5 + 32;                    // Fahrenheit

  return Tc;
}

float get_freq(uint32_t time_us) {
  float freq = 0;

  if (time_us != 0) {
    freq = 100000;
    freq /= time_us;
    freq /= 2;
  }

  return freq;
}

void disp_set_deg(int8_t temp, bool power, bool heater) {
  bool is_negative = false;

  if (temp < 0) {
    is_negative = true;
    temp *= -1;
  }

  uint8_t n10 = temp / 10;
  uint8_t n1 = temp - (n10 * 10);

  if ((temp >= 0) && (temp <= 99) && (is_negative == false)) {
    disp_buff[0] = display.encodeDigit(n10);
    disp_buff[1] = display.encodeDigit(n1);
  } else if ((temp <= 9) && (is_negative == true)) {
    disp_buff[0] = 0b1000000;  // (-)
    disp_buff[1] = display.encodeDigit(n1);
  } else if (is_negative == false) {
    disp_buff[0] = 0b1111000;  // t
    disp_buff[1] = 0b1110110;  // H
  } else {
    disp_buff[0] = 0b1111000;  // t
    disp_buff[1] = 0b0111000;  // L
  }

  disp_buff[2] = 0b01100011 | (power << 7);   // deg °
  disp_buff[3] = 0b00111001 | (heater << 7);  // C
}