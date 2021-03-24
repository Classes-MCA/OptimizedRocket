% Purpose: to take a few spline points and convert them into a set of
% trajectory points for use in the trajectory function and in the other
% parts of the optimization code.
%
% Inputs:
%   - splinePoints: the few points for which the spline will be performed.
%                   These are given as (x,y) coordinates
%
% Outputs:
%   - x: the x-locations of the trajectory after an interpolation has been
%        performed on the spline
%   - y: the y-locations of the trajectory after an interpolation has been
%        performed on the spline

function [x,y] = splineToTrajectory(splinePoints)

    x_spline = splinePoints(:,1);
    y_spline = splinePoints(:,2);
    
    deltaY = 100;
    
    y = 0:deltaY:max(y_spline);
    x = makima(y_spline,x_spline,y);
    %plot(x_spline,y_spline,'o',x,y,'--')
    


end