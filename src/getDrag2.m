% Purpose: To calculate the drag on the vehicle
%
% Inputs:
%   - rocket: a MATLAB rocket object
%
% Outputs:
%   - drag: the total drag force on the rocket in Newtons

function drag = getDrag2(velocity,height)
    
    % Calculate  information about the atmosphere
    atmosphere = getAtmosphere(height);
    density = atmosphere.Density;
    
    if real(velocity) < atmosphere.SpeedOfSound % If subsonic
        % Subsonic drag
        drag = log10(velocity * density + 1) * 1e6;
        
    else % If supersonic
        % Drag increases a lot
        drag = log10(velocity * density + 1) * 1e6;    
    end
    
    if real(height) >= 25e3 && real(height) < 50e3
        drag = 1e7;
    else
        drag = 0;
    end

    % drag = 1e6 ./ (height - 1)^0.2;
    
end