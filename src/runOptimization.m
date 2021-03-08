function [xopt, fopt, iter, opt] = runOptimization()

    x0 = logspace(0,4,20);
    %dx = 10e2/20;
    %x0 = 0:dx:(10e2-dx);
    lb = zeros(1,length(x0));
    ub = [];
    targetY = 10e3; % meters
    deltaY = targetY / length(x0); % meters
    y = 0:deltaY:targetY - deltaY;

    function [f, c, ceq] = objcon(x)
        
        usedMass = trajectory(x);
        f = usedMass;
        
        subtractedValues = circshift(x,1) - x - 2;
        c = subtractedValues(2:end);
        
        %c = c ./ max(abs(c));
        
        ceq = x(1);
    end

    options = optimoptions(@fmincon,...
                           'Display','iter-detailed',...
                           'OutputFcn',@outfun,...
                           'ScaleProblem',false,...
                           'MaxFunctionEvaluations',10e4);
        
    iter = [];
    opt = [];
                       
    function stop = outfun(x,optimValues,state)
        iter = [iter;optimValues.iteration];
        opt = [opt;optimValues.firstorderopt];
        stop = false; % you can set your own stopping criteria
    end

    %----------------------------------------------
    % Things below this line don't need to be changed

    xlast = [];
    flast = [];
    clast = [];
    ceqlast = [];

    [xopt, fopt] = fmincon(@objective,x0,[],[],[],[],lb,ub,@con,options);
    
    figure()
    plot(xopt./1000,y./1000)
    title("Trajectory")
    xlabel("X (km)")
    ylabel("Y (km)")

    function f = objective(x)
        
        if ~isequal(x,xlast)
            
            [flast, clast, ceqlast] = objcon(x);
            
            xlast = x;
            
        end
        
        f = flast;
        
    end

    function [c, ceq] = con(x)

        if ~isequal(x,xlast)
            
            [flast, clast, ceqlast] = objcon(x);
            
            xlast = x;
            
        end
        
        c = clast;
        ceq = ceqlast;
        
    end

end