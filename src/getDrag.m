% Purpose: To calculate the drag on the vehicle
%
% Inputs:
%   - rocket: a MATLAB rocket object
%
% Outputs:
%   - drag: the total drag force on the rocket in Newtons

function drag = getDrag(rocket)

    velocity = norm(rocket.Velocity);
    height = rocket.Location(2);
    
    drag = log10(velocity / height + 1) * 1000;

end