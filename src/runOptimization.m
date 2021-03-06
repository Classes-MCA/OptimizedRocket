clear; close all;

x0 = logspace(0,5,272) - 1;
targetY = 250e3; % meters
deltaY = targetY / length(x0); % meters
y = 0:deltaY:targetY - deltaY;

options = optimoptions(@fminunc,...
                       'Display','iter-detailed',...
                       'MaxFunctionEvaluations',1e5,...
                       'MaxIterations',1e3);
[xstar,fstar] = fmincon(@objective,x0,options);

figure()
% subplot(2,1,1)
plot(xstar./1000,y./1000)
title("Trajectory")
xlabel("X (km)")
ylabel("Y (km)")

function [f, c, ceq] = objcon(x)
        [mass, stress] = truss(x);
        f = mass;
        c = stress.^2 - stressmax.^2; % scale
        
        c = c./1000; % scaling
        
        ceq = [];
end
    
