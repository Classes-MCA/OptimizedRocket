% Purpose: To calculate the drag on the vehicle
%
% Inputs:
%   - rocket: a MATLAB rocket object
%
% Outputs:
%   - drag: the total drag force on the rocket in Newtons

function drag = getDrag(rocket)

    % Extract information about the rocket's speed and altitude
    velocity = norm(rocket.Velocity);
    height = rocket.Location(2);
    
    % Calculate  information about the atmosphere
    atmosphere = getAtmosphere(height);
    density = atmosphere.Density;
    
    if velocity < atmosphere.SpeedOfSound
        % Subsonic drag
        drag = log10(velocity * density + 1) * 1000;
        
    else
        % Drag increases a lot
        drag = log10(velocity * density + 1) * 2000;
        
    end

end