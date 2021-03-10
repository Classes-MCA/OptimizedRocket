function [xopt, fopt, exitflag, output] = runOptimization2()

    % -------- starting point and bounds ----------
    downrangeDistance = 50e3; % meters
    xPoints = 100;
    exitAngle = 20; % Degrees
    dx = downrangeDistance/xPoints;
    x0 = 0:dx:downrangeDistance;
    %x0 = logspace(0,log10(downrangeDistance),xPoints+1);
    
    targetY = 100e3; % meters
    deltaY = targetY / length(x0); % meters
    y = 0:deltaY:targetY - deltaY;
    
    % Making all of the x-inputs be of a similar order
    x0 = log(x0(2:end)); % Removing the first zero
    %x0 = x0 + rand(1,length(x0))./10;
    
    lb = zeros(1,length(x0));
    lb = [];
    ub = log(ones(1,length(x0)) .* downrangeDistance + 0.01);
    
    
    % ---------------------------------------------

    % ------ linear constraints ----------------
    % We won't be doing this for the truss example. Leave these as empty
    % arrays
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    
    % Linear Inequality Constraints
    A = zeros(length(x0));
    
    for k = 1:length(x0)
    
        A(k,k) = 1;
        A(k,k+1) = -1;
    
    end
    
    A = A(1:end-1,:);
    
    for k = 1:length(x0)
    
        A(length(x0) + k,k) = 5;
        A(length(x0) + k,k+1) = -5;
        A(length(x0) + k,k+2) = -1;
        A(length(x0) + k,k+3) = 1;
    
    end
    
    A = A(:,1:length(x0));
    A = A(1:end-1,:);
    
    b = zeros(length(A(:,1)),1);
    
    % Linear Equality Constraints
%     for k = 1:length(x0)-3
%     
%         A(k,k) = 0;
%         A(k,k+1) = 0;
%         A(k,k+2) = 0;
%         A(k,k+3) = 0;
%     
%     end
    
    % ------------------------------------------
    
    % Counting the number of function calls
    global funcCount
    funcCount = 0;

    % ---- Objective and Constraints -------------
    function [f, g, h, df, dg, dh] = objcon(x)

        x = exp(x);
        
        % set objective/constraints here
        % f: objective
        % g: inequality constraints
        % h: equality constraints
        % d*: derivatives of * (see fmincon docs for index ordering)
        
        % Derivative Method
        method = 'Complex-Step';
        
        % Interpretation
        % f: the objective value
        f = trajectory(x);
        f = f.usedMass;
        
        % g: inequality constraints
        constraints = inequalityConstraints(x);
        g = constraints.inequalityConstraints;
        
        % h: equality constraints, see first homework. There are none for
        % this homework
        constraints = equalityConstraints(x);
        h = constraints.equalityConstraints;
        
        % df: simple derivative
        J = getJacobian(@trajectory,x,...
                        'Method',method);
                    
        df = J(1).output;
        
        % dg: Jacobian of g.
        J = getJacobian(@inequalityConstraints,x,...
                        'Method',method);
        dg = J(1).output;
        
        % dh: Jacobian of h
        J = getJacobian(@equalityConstraints,x,...
                        'Method',method);
        dh = J(1).output;

    end

    %--- EQUALITY CONSTRAINTS
    function constraints = equalityConstraints(x)
        
        % Setting the endpoint
        ceq = x(end) - downrangeDistance;
        
        % Setting the rocket tilt angle at the endpoint
%         deltaX = x(end) - x(end-1);
%         angle = atan(deltaX/deltaY) * 180/pi;
%         ceq = [ceq; exitAngle - angle];

        constraints.equalityConstraints = ceq(:);
        
    end

    %--- INEQUALITY CONSTRAINTS
    function constraints = inequalityConstraints(x)
        
        c = [];
        
        % Each x-value must be larger than the previous (ie. forward
        % motion)
%         subtractedValues = circshift(x,1) - x;
%         c = subtractedValues(2:end);
%         c = c(:);
        
        % Adding the constraint that each successive delta x has to be less
        % than double the previous delta x
%         for i = 3:length(x)
%             
%             c = [c; (x(i) - x(i-1) - 10*(x(i-1) - x(i-2)))];
%             
%         end
        
        % The angles between iterations can't be too steep
%         for i = 3:length(x)
%             
%             deltaXprevious = x(i-1) - x(i-2);
%             deltaXcurrent = x(i) - x(i-1);
%             thetaPrevious = atan(deltaXprevious/deltaY) * 180/pi;
%             thetaCurrent = atan(deltaXcurrent/deltaY) * 180/pi;
%             c = [c; thetaCurrent - thetaPrevious - 30];
%             
%         end
        
        % Each point cannot be larger than the downrange distance
        % c = [c; (x - downrangeDistance).'];
        
        constraints.inequalityConstraints = c(:);
        
    end

    %---- TRAJECTORY FUNCTION
    function flight = trajectory(x)
        
        % Forcing the optimizer to meet these requirements for the end points
        x = [0,x]; % Putting the zero back in there

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
        'OptimalityTolerance', 1e-6, ...  % convergence tolerance on first order optimality
        'ConstraintTolerance', 1e-6, ...  % convergence tolerance on constraints
        'FiniteDifferenceType', 'central', ...  % if finite differencing, can also use central
        'SpecifyObjectiveGradient', true, ...  % supply gradients of objective
        'SpecifyConstraintGradient', true, ...  % supply gradients of constraints
        'CheckGradients', false, ...  % true if you want to check your supplied gradients against finite differencing
        'Diagnostics', 'on',... % display diagnotic information
        'OutputFcn',@outfun,...
        'StepTolerance',1e-12,...
        'FunctionTolerance',1e-12,...
        'OptimalityTolerance',1e-6);
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
        
        downrangeValues = [0,exp(x)];
        
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
    [xopt, fopt, exitflag, output] = fmincon(@obj, x0, A, b, Aeq, beq, lb, ub, @con, options);

    output.opt = opt;
    output.funcCallCount = funcCallCount;
    
%     figure()
%     plot(exp(xopt)./1000,y./1000)
%     title("Trajectory")
%     xlabel("X (km)")
%     ylabel("Y (km)")
    
end
