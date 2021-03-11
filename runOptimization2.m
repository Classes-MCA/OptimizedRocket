function [xopt, fopt, exitflag, output] = runOptimization2()

    % -------- starting point and bounds ----------
    downrangeDistance = 15e3; % meters
    xPoints = 60;
    exitAngle = 70; % Degrees
    dx = downrangeDistance/xPoints;    
    deltaX0 = ones(xPoints,1) * dx;
    
    targetY = 50e3; % meters
    deltaY = targetY / (length(deltaX0)+1); % meters
    y = 0:deltaY:targetY - deltaY;
    
    lb = [];
    ub = [];
    
    
    % ---------------------------------------------

    % ------ linear constraints ----------------
    
    %--- Linear Inequality Constraints
    
    A = []; b = [];
    
%     A = zeros(length(deltaX0));
%     
%     % Make each point be further downrange than the one before it
%     for j = 1:length(deltaX0)
%     
%         A(j,j) = 1;
%         A(j,j+1) = -1;
%     
%     end
%     
%     A = A(1:end-1,:);
%     
% %     % Making each successive difference between points be no more than 'q'
% %     % times the previous difference
% %     for j = 1:length(deltaX0)-2
% %         
% %         q = 2;
% %     
% %         A(j + length(deltaX0)-1,j) = q;
% %         A(j + length(deltaX0)-1,j+1) = -1 - q;
% %         A(j + length(deltaX0)-1,j+2) = 1;
% %     
% %     end
%     
%     A = A(:,1:length(deltaX0));
%     
%     b = zeros(length(A(:,1)),1) - 10;

    Aeq = []; beq = [];
    
    %--- Linear Equality Constraints
    
%     % Set the endpoints
%     Aeq = zeros(2,length(x0));
%     Aeq(1,1) = 1;
%     Aeq(2,end) = 1;
%     beq(1) = 0;
%     beq(2) = downrangeDistance;
  
    % ------------------------------------------
    
    % Counting the number of function calls
    global funcCount
    funcCount = 0;

    % ---- Objective and Constraints -------------
    function [f, g, h, df, dg, dh] = objcon(deltaX)
        
        % set objective/constraints here
        % f: objective
        % g: inequality constraints
        % h: equality constraints
        
        % Derivative Method
        method = 'Complex-Step';
        
        % Interpretation
        % f: the objective value
        f = trajectory(deltaX);
        f = f.usedMass;

        % df: simple derivative
        J = getJacobian(@trajectory,deltaX,...
                        'Method',method);
                    
        df = J(1).output;
        
        g = [];
        dg = [];
        
        h = [];
        dh = [];
        
%         % g: inequality constraints
%         constraints = inequalityConstraints(deltaX);
%         g = constraints.inequalityConstraints;
%         
%         % dg: Jacobian of g.
%         J = getJacobian(@inequalityConstraints,deltaX,...
%                         'Method',method);
%         dg = J(1).output;
        
        % h: equality constraints, see first homework.
        constraints = equalityConstraints(deltaX);
        h = constraints.equalityConstraints;
        
        % dh: Jacobian of h
        J = getJacobian(@equalityConstraints,deltaX,...
                        'Method',method);
        dh = J(1).output;
               
    end

    %--- EQUALITY CONSTRAINTS
    function constraints = equalityConstraints(deltaX)
        
        ceq = [];
        
        x = generateX(deltaX);
        
        ceq(1) = x(1);
        ceq(2) = x(end) - downrangeDistance;
        
        constraints.equalityConstraints = ceq;
        
    end

    %--- INEQUALITY CONSTRAINTS
    function constraints = inequalityConstraints(deltaX)
        
        c = [];
        
        x = generateX(deltaX);
        
        for i = 2:length(x)
            
            c = [c;x(i-1) - x(i) + 10];
            
        end
        
        constraints.inequalityConstraints = c;
        
    end

    %--- GENERATE X-ARRAY
    function x = generateX(deltaX)
        
        x = [0];
        for i = 2:length(deltaX)
            
            x(i) = x(i-1) + deltaX(i-1);
            
        end
        
        x(end+1) = downrangeDistance;
        
    end

    %---- TRAJECTORY FUNCTION
    function flight = trajectory(deltaX)
        
        x = generateX(deltaX);

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
            height = (k-1) * deltaY;

            % Calculate the tilt angle
            theta = atan(deltaX/deltaY);

            % Calculate the drag
            D = getDrag2(velocity(k-1),height); % simple model (not physics-based)

            % calculate the thrust
            Tx = (mass(k-1) * a + D) * sin(theta);
            Ty = (mass(k-1) * a + D) * cos(theta) + mass(k-1) * g;
            T = sqrt(Tx^2 + Ty^2);

            % calculate the change in mass
            deltaTime = sqrt(deltaX^2 + deltaY^2) / velocity(k-1);
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
        'MaxFunctionEvaluations', 10000, ...  % maximum number of function calls
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
    function stop = outfun(deltaX,optimValues,state)
        opt = [opt;optimValues.firstorderopt];
        if ~isempty(optimValues.firstorderopt)
            funcCallCount = [funcCallCount;funcCount];
        end
        stop = false; % you can set your own stopping criteria
        
        downrangeValues = generateX(deltaX);
        
        plot(downrangeValues/1000,y./1000,'*')
        title(strcat("Trajectory, Final Rocket Mass = ",sprintf('%03.0f',1e6 - optimValues.fval*1e5)," kg"))
        xlabel("X (km)")
        ylabel("Y (km)")
        %axis equal
        xlim([0,(downrangeDistance/1000)*1.1])
        ylim([0,targetY/1000 * 1.1])
        drawnow()
        
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
    [xopt, fopt, exitflag, output] = fmincon(@obj, deltaX0, A, b, Aeq, beq, lb, ub, @con, options);

    output.opt = opt;
    output.funcCallCount = funcCallCount;
    
%     figure()
%     plot(exp(xopt)./1000,y./1000)
%     title("Trajectory")
%     xlabel("X (km)")
%     ylabel("Y (km)")
    
end
