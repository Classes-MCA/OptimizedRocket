% Purpose: To add all of the code from the 'OptimizeRocket' project to your
% current MATLAB path so that you can use code from each folder in the
% project at once
%
% Here's how to use it
%   1. Make sure that you're within the 'OptimizeRocket' project (any
%   folder will do)
%   2. Type 'getRocketCode' at the top of your script or in the command
%   window
%   3. That's it! You now have access to all the code in the project!
%
% Note: make sure that no two functions have the same name, otherwise
% they'll conflict.


function getRocketCode()

    currentPath = pwd;
    
    index = findstr(currentPath,'OptimizedRocket');
    
    rocketPath = currentPath(1:index + 15);
    
    allPaths = genpath(rocketPath);
    
    addpath(allPaths)

end