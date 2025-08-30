# EHC2025
Electric heater controller 2025 - For hot-cold water tap - ESP32 based with dual YF-S201 flow sensor

- Work in progress.

>   (Source: Google IA)
> 	To calculate the shower water outlet temperature,
> 	you need to consider the heater power,
> 	the water flow rate, and the initial water temperature.
> 	The formula for calculating the temperature change is
> 	ΔT = (Power * Time) / (Mass * Specific Heat).
> 	However, since the flow rate is given in liters per minute,
> 	we need to convert to units compatible with the
> 	power and specific heat of the water.
> 
> 	Calculation steps:
> 
> 	1. Convert flow rate to compatible units:
> 	2 liters per minute is equivalent to 2/60 = 0.0333 liters per second.
> 	The density of water is approximately 1 kg/liter, so 0.0333 liters/s = 0.0333 kg/s.
> 	The specific heat of water is approximately 4186 J/(kg°C).
> 
> 	2. Calculate the heat added to the water:
> 	The shower's power is 5400 watts, which translates to 5400 joules per second.
> 	To calculate the temperature change in 1 second, we use: ΔT = (Power * Time) / (Mass * Specific Heat).
> 	ΔT = (5400 W * 1 s) / (0.0333 kg * 4186 J/(kg °C))
> 	ΔT ≈ 38.9 °C
> 
> 	3. Calculate the final water temperature:
> 	The initial water temperature is 25°C.
> 	The calculated temperature change is approximately 38.9°C.
> 	The final water temperature will be: 25°C + 38.9°C = 63.9°C
