function [xopt, fopt, exitflag, output] = runOptimization2()

    % -------- starting point and bounds ----------
    downrangeDistance = 250e3; % meters
    xPoints = 10;
    exitAngle = 60;
    dx = downrangeDistance/xPoints;
    x0 = 0:dx:downrangeDistance;
    %x0 = [0,1000 3000 6000 10000];
    x0 = log(x0(2:end));
    
    targetY = 250e3; % meters
    deltaY = targetY / (length(x0)+1); % meters
    y = 0:deltaY:targetY - deltaY;
    
    % Making all of the x-inputs be of a similar order
    
    lb = [];
    ub = [];
    
    % ---------------------------------------------

    % ------ linear constraints ----------------
    
    %--- Linear Inequality Constraints
    A = zeros(length(x0));
    
    % Make each point be further downrange than the one before it
    for j = 1:length(x0)
    
        A(j,j) = 1;
        A(j,j+1) = -1;
    
    end
    
    A = A(1:end-1,:);
    
    % Making each successive difference between points be larger
    for j = 1:length(x0)-2
    
        A(j + length(x0)-1,j) = -1;
        A(j + length(x0)-1,j+1) = 2;
        A(j + length(x0)-1,j+2) = -1;
    
    end
    
    A = A(:,1:length(x0));
    
    b = zeros(length(A(:,1)),1);
    %b(1:length(x0)-1) = ones(length(x0)-1,1);

    %A = []; b = [];
    
    %--- Linear Equality Constraints
    
    % Set the endpoint to be at the desired downrange location
    Aeq = zeros(1,length(x0));
    Aeq(end) = 1;
    beq = log(downrangeDistance);
    
    %Aeq = []; beq = [];
  
    % ------------------------------------------
    
    % Counting the number of function calls
    global funcCount
    funcCount = 0;

    % ---- Objective and Constraints -------------
    function [f, g, h, df, dg, dh] = objcon(x)
        
        % set objective/constraints here
        % f: objective
        % g: inequality constraints
        % h: equality constraints
        
        % Derivative Method
        method = 'Complex-Step';
        
        % Interpretation
        % f: the objective value
        f = trajectory(x);
        f = f.usedMass;

        % df: simple derivative
        J = getJacobian(@trajectory,x,...
                        'Method',method);
                    
        df = J(1).output;
        
        g = [];
        dg = [];
        
        h = [];
        dh = [];
        
        % g: inequality constraints
        constraints = inequalityConstraints(x);
        g = constraints.inequalityConstraints;
        
        % dg: Jacobian of g.
        J = getJacobian(@inequalityConstraints,x,...
                        'Method',method);
        dg = J(1).output;
        
        % h: equality constraints
        constraints = equalityConstraints(x);
        h = constraints.equalityConstraints;
        
        % dh: Jacobian of h
        J = getJacobian(@equalityConstraints,x,...
                        'Method',method);
        dh = J(1).output;
               
    end

    %--- EQUALITY CONSTRAINTS
    function constraints = equalityConstraints(x)
        
        x = exp(x);
        x = [0,x];
        splinePoints = [x.',y.'];
        [x,y_interpolated] = splineToTrajectory(splinePoints);
        
        ceq = [];
        
        % Setting the exit angle
        endDeltaX = x(end) - x(end-1);
        endDeltaY = y_interpolated(end) - y_interpolated(end-1);
        theta = atan(endDeltaX/endDeltaY) * 180/pi;
        ceq = [ceq,theta - exitAngle];
        
        constraints.equalityConstraints = ceq;
        
    end

    %--- INEQUALITY CONSTRAINTS
    function constraints = inequalityConstraints(x)
        
        x = exp(x);
        x = [0,x];
        splinePoints = [x.',y.'];
        [x,y_interpolated] = splineToTrajectory(splinePoints);
        
        c = [];
        
        % Setting start angle
        startDeltaX = x(2) - x(1);
        startDeltaY = y_interpolated(2) - y_interpolated(1);
        startAngle = atan(startDeltaX/startDeltaY) * 180/pi;
        startAngleMin = 5;
        
        c = [c,startAngle - startAngleMin];
        
        % Maximum turn between points
        maxTurn = 1; % Degrees
        
        for i = 3:length(x)
            
            currentDeltaX = x(i) - x(i-1);
            currentDeltaY = y_interpolated(i) - y_interpolated(i-1);
            
            previousDeltaX = x(i-1) - x(i-2);
            previousDeltaY = y_interpolated(i-1) - y_interpolated(i-2);
            
            currentTheta = atan(currentDeltaX/currentDeltaY) * 180/pi;
            previousTheta = atan(previousDeltaX/previousDeltaY) * 180/pi;
            
            c = [c,currentTheta - previousTheta - maxTurn];
            
        end
        
        constraints.inequalityConstraints = c;
        
    end

    %---- TRAJECTORY FUNCTION
    function flight = trajectory(x)
        
        x = exp(x);
        x = [0,x];
        splinePoints = [x.',y.'];
        [x,y_interpolated] = splineToTrajectory(splinePoints);
        deltaY_interpolated = y_interpolated(2) - y_interpolated(1);
        
        % Forcing the optimizer to meet these requirements for the end points
        %x = [0,x]; % Putting the zero back in there

        fuelExitVelocity = 2000; % meters/second

        g = 9.8; % m/s^2
        a = 3*g; % m/s^2

        mass = zeros(1,length(x));
        mass(1) = 1e6; % initial mass

        velocity = zeros(1,length(x));
        velocity(1) = 50;

        time = zeros(1,length(x));

        % Iterate over each x-position
        for k = 2:length(x)

            % Getting initial conditions going for this round
            deltaX = x(k) - x(k-1);
            height = (k-1) * deltaY_interpolated;

            % Calculate the tilt angle
            theta = atan(deltaX/deltaY_interpolated);

            % Calculate the drag
            D = getDrag2(velocity(k-1),height); % simple model (not physics-based)

            % calculate the thrust
            Tx = (mass(k-1) * a + D) * sin(theta);
            Ty = (mass(k-1) * a + D) * cos(theta) + mass(k-1) * g;
            T = sqrt(Tx^2 + Ty^2);

            % calculate the change in mass
            deltaTime = sqrt(deltaX^2 + deltaY_interpolated^2) / velocity(k-1);
            deltaMass = T / fuelExitVelocity * deltaTime;

            % Update some values for the next round
            velocity(k) = velocity(k-1) + a * deltaTime;
            mass(k) = mass(k-1) - deltaMass;
            time(k) = time(k-1) + deltaTime;

        end

        usedMass = mass(1) - mass(end);

        flight.usedMass = usedMass / 1e6;
    
    end

    % -------------------------------------------

    % ----------- options ----------------------------
    options = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', ...  % choose one of: 'interior-point', 'sqp', 'active-set', 'trust-region-reflective'
        'HonorBounds', true, ...  % forces optimizer to always satisfy bounds at each iteration
        'Display', 'iter-detailed', ...  % display more information
        'MaxIterations', 1000, ...  % maximum number of iterations
        'MaxFunctionEvaluations', 1000, ...  % maximum number of function calls
        'OptimalityTolerance', 1e-9, ...  % convergence tolerance on first order optimality
        'ConstraintTolerance', 1e-6, ...  % convergence tolerance on constraints
        'FiniteDifferenceType', 'central', ...  % if finite differencing, can also use central
        'SpecifyObjectiveGradient', true, ...  % supply gradients of objective
        'SpecifyConstraintGradient', true, ...  % supply gradients of constraints
        'CheckGradients', false, ...  % true if you want to check your supplied gradients against finite differencing
        'Diagnostics', 'on',... % display diagnotic information
        'OutputFcn',@outfun,...
        'StepTolerance',1e-16,...
        'FunctionTolerance',1e-12,...
        'ScaleProblem',true);
    % -------------------------------------------
    
    opt = [];
    funcCallCount = [];
    
    % Creating an output function
    function stop = outfun(x,optimValues,state)
        opt = [opt;optimValues.firstorderopt];
        if ~isempty(optimValues.firstorderopt)
            funcCallCount = [funcCallCount;funcCount];
        end
        stop = false; % you can set your own stopping criteria
        
        x = exp(x);
        x = [0,x];
        
        splinePoints = [x',y'];
        [x_current,y_current] = splineToTrajectory(splinePoints);
        
        plot(x_current./1000,y_current./1000,'-'); hold on
        plot(x./1000,y./1000,'*')
        title(strcat("Trajectory, Final Rocket Mass = ",sprintf('%03.0f',1e6 - optimValues.fval*1e5)," kg"))
        xlabel("X (km)")
        ylabel("Y (km)")
        %axis equal
        xlim([0,(downrangeDistance/1000)*1.1])
        ylim([0,targetY/1000 * 1.1])
        ax = gca;
        ax.Title.FontSize = 16;
        ax.XAxis.FontSize = 14;
        ax.YAxis.FontSize = 14;
        ax.Parent.Position = [2 2 6.5 5];
        drawnow()
        hold off
        
    end


    % -- NOTE: no need to change anything below) --
    % see: https://www.mathworks.com/help/optim/ug/objective-and-nonlinear-constraints-in-the-same-function.html
    

    % ------- shared variables -----------
    xlast = [];  % last design variables
    flast = [];  % last objective
    glast = [];  % last nonlinear inequality constraint
    hlast = [];  % last nonlinear equality constraint
    dflast = [];  % last derivatives
    dglast = [];
    dhlast = [];
    % --------------------------------------


    % ------ separate obj/con  --------
    function [f, df] = obj(x)

        % check if computation is necessary
        if ~isequal(x, xlast)
            [flast, glast, hlast, dflast, dglast, dhlast] = objcon(x);
            xlast = x;
        end

        f = flast;
        df = dflast;
    end

    function [g, h, dg, dh] = con(x)

        % check if computation is necessary
        if ~isequal(x, xlast)
            [flast, glast, hlast, dflast, dglast, dhlast] = objcon(x);
            xlast = x;
        end

        % set constraints
        g = glast;
        h = hlast;
        dg = dglast;
        dh = dhlast;
    end
    % ------------------------------------------------

    % call fmincon
    [xopt, fopt, exitflag, output] = fmincon(@obj, x0, A, b, Aeq, beq, lb, ub, @con, options);

    output.opt = opt;
    output.funcCallCount = funcCallCount;
    
%     figure()
%     plot(exp(xopt)./1000,y./1000)
%     title("Trajectory")
%     xlabel("X (km)")
%     ylabel("Y (km)")
    
end
