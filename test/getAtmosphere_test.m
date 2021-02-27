clear; close all;

height = 0:1:100e3;

atmosphere = getAtmosphere(height)

figure()
plot(atmosphere.Temperature,height)
title('Standard Atmosphere: Temperature')
xlabel('Temperature (K)')
ylabel('Height (meters)')

figure()
plot(atmosphere.Pressure,height)
title('Standard Atmosphere: Pressure')
xlabel('Pressure (Pa)')
ylabel('Height (meters)')

figure()
plot(atmosphere.Density,height)
title('Standard Atmosphere: Density')
xlabel('Density (kg/m^3)')
ylabel('Height (meters)')

figure()
plot(atmosphere.SpeedOfSound,height)
title('Standard Atmosphere: Sound Speed')
xlabel('Speed of Sound (m/s)')
ylabel('Height (meters)')