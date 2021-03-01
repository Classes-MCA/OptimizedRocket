clear; close all;

height = 0:1:250e3;

atmosphere = getAtmosphere(height)

figure()
plot(atmosphere.Temperature,height./1000)
title('Standard Atmosphere: Temperature')
xlabel('Temperature (K)')
ylabel('Height (km)')

figure()
plot(atmosphere.Pressure,height./1000)
title('Standard Atmosphere: Pressure')
xlabel('Pressure (Pa)')
ylabel('Height (km)')

figure()
plot(atmosphere.Density,height./1000)
title('Standard Atmosphere: Density')
xlabel('Density (kg/m^3)')
ylabel('Height (km)')

figure()
plot(atmosphere.SpeedOfSound,height./1000)
title('Standard Atmosphere: Sound Speed')
xlabel('Speed of Sound (m/s)')
ylabel('Height (km)')