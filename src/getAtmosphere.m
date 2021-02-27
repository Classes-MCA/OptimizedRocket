% Purpose: to get the atmospheric properties of the air at a certain
% altitude
%
% Inputs:
%   - height: the height of the object, in meters
%
% THIS MODEL IS ONLY OFFICIALLY GOOD UP TO 42 KM. WE WILL USE IT UP TO FULL
% ALTITUDE BECAUSE PRESSURE AND DENSITY DO GO TO ZERO.

function atmosphere = getAtmosphere(height)

    Tsl = 300; % Temperature (K) at sea level
    psl = 101325; % Pressure (Pa) at sea level
    R = 287.053; % Specific gas constant for air (J/(kg*K))
    h = height./1000; % Converting to kilometers
    maxHeight = 60; % The normal model fails above 42 km. We'll apply our
                    % corrections officially starting at this alitude
    gamma = 1.4; % Ratio of specific heats for air
    
    atmosphere.Temperature = Tsl - 71.5 + 2 .* log(1 + exp(35.75 - 3.25*h) +...
                             exp(-3 + 0.0003.*h.^3));
                      
    highAltitudeCorrection = 0.5 .* (1 + 2/pi .* atan(-h/5 + maxHeight/5));
                         
    atmosphere.Temperature = atmosphere.Temperature .* highAltitudeCorrection;
                         
    atmosphere.Pressure = psl .* exp(-0.118.*h - 0.0015.*h.^2 ./ ...
                          (1 - 0.018.*h + 0.0011.*h.^2));
                      
    atmosphere.Density = atmosphere.Pressure ./ (R .* atmosphere.Temperature);

    atmosphere.GasConstant = R;
    
    atmosphere.SpeedOfSound = sqrt(gamma .* R .* atmosphere.Temperature);
    
end