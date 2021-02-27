function [xopt, fopt, exitflag, output] = runOptimization()

    % -------- starting point and bounds ----------
    x0 = [];
    ub = [];
    lb = [];
    % ---------------------------------------------

    % ------ linear constraints ----------------
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    % ------------------------------------------

    % ---- Objective and Constraints -------------
    function [f, g, h, df, dg, dh] = objcon(x)

        % set objective/constraints here
        % f: objective
        % g: inequality constraints
        % h: equality constraints
        % d*: derivatives of * (see fmincon docs for index ordering)

    end
    % -------------------------------------------

    % ----------- options ----------------------------
    options = optimoptions('fmincon', ...
        'Algorithm', 'active-set', ...  % choose one of: 'interior-point', 'sqp', 'active-set', 'trust-region-reflective'
        'HonorBounds', true, ...  % forces optimizer to always satisfy bounds at each iteration
        'Display', 'iter-detailed', ...  % display more information
        'MaxIterations', 1000, ...  % maximum number of iterations
        'MaxFunctionEvaluations', 10000, ...  % maximum number of function calls
        'OptimalityTolerance', 1e-6, ...  % convergence tolerance on first order optimality
        'ConstraintTolerance', 1e-6, ...  % convergence tolerance on constraints
        'FiniteDifferenceType', 'forward', ...  % if finite differencing, can also use central
        'SpecifyObjectiveGradient', false, ...  % supply gradients of objective
        'SpecifyConstraintGradient', false, ...  % supply gradients of constraints
        'CheckGradients', false, ...  % true if you want to check your supplied gradients against finite differencing
        'Diagnostics', 'on');  % display diagnotic information
    % -------------------------------------------


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

end
