clear; close all;

height = 0:1:42e3;

atmosphere = getAtmosphere(height)

h = figure();
plot(atmosphere.Temperature,height./1000)
title('Standard Atmosphere - Temperature')
xlabel('Temperature (K)')
ylabel('Height (km)')
prettyPlot(h)

h = figure();
plot(atmosphere.Pressure,height./1000)
title('Standard Atmosphere - Pressure')
xlabel('Pressure (Pa)')
ylabel('Height (km)')
prettyPlot(h)

h = figure();
plot(atmosphere.Density,height./1000)
title('Standard Atmosphere - Density')
xlabel('Density (kg/m^3)')
ylabel('Height (km)')
prettyPlot(h)

h = figure();
plot(atmosphere.SpeedOfSound,height./1000)
title('Standard Atmosphere - Sound Speed')
xlabel('Speed of Sound (m/s)')
ylabel('Height (km)')
prettyPlot(h)

function prettyPlot(h)

    h.Children(1).XAxis.FontSize = 15;
    h.Children(1).YAxis.FontSize = 15;
    h.Children(1).Title.FontSize = 17;
    h.Children(1).Children.LineWidth = 4
    h.Position = [2 2 6.5 5]

end