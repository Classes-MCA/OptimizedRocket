function J = getJacobian(func,x,varargin)
    
    tic; % Start counting time

    p = inputParser;
    p.addRequired('func');
    p.addRequired('x');
    p.addParameter('Method','Finite-Difference')
    p.addParameter('StepSize',1e-8)
    
    p.parse(func,x,varargin{:});
    Method = p.Results.Method;
    h = p.Results.StepSize;
    
    % Setting dimensions for the Jacobian
    inputNum = length(x);
    outputNum = nargout(func);
    
    global funcCount % Use this variable here too
    
    for i = 1:inputNum
            
            if strcmp(lower(Method),lower('Finite-Difference'))
                
                % Do a central-difference
                x_forward = x;
                x_backward = x;
                x_forward(i) = x_forward(i) + h/2;
                x_backward(i) = x_backward(i) - h/2;
                
                output_forward = func(x_forward); funcCount = funcCount + 1;
                output_backward = func(x_backward); funcCount = funcCount + 1;
                
                output_vars = fieldnames(output_forward);
                
                % Create a Jacobian for each function output
                for j = 1:length(output_vars)
                    
                    forward = eval(['output_forward.',output_vars{j}]);
                    backward = eval(['output_backward.',output_vars{j}]);
                    
                    deriv = (forward - backward)/h;
                    
                    J(j).output(i,:) = deriv;
                    
                end
                
            elseif strcmp(lower(Method),lower('Complex-Step'))
                
                % Define small step size
                h = 1e-30;
                
                % Do a central-difference
                x_step = x;
                x_step(i) = x_step(i) + 1j*h;
                
                output_step = func(x_step); funcCount = funcCount + 1;
                
                output_vars = fieldnames(output_step);
                
                % Create a Jacobian for each function output
                for k = 1:length(output_vars)
                    
                    step = eval(['output_step.',output_vars{k}]);
                    
                    deriv = imag(step)/h;
                    
                    J(k).output(i,:) = deriv;
                    
                end

            elseif strcmp(lower(Method),lower('AD'))
                
                if i > 1
                    x = x{0}; % Convert back to double by taking only
                              % zeroth-order derivatives
                end
                
                x = amatinit(x);
                output_AD = func(x); funcCount = funcCount + 1;
                
                output_vars = fieldnames(output_AD);
                
                % Create a Jacobian for each function output
                for k = 1:length(output_vars)
                    
                    value = eval(['output_AD.',output_vars{k}]);
                    
                    jacobian = ajac(value);
                    
                    J(k).output = transpose(jacobian{0});
                    
                end
                
            end
        
    end
    
    J(3).output = toc; % Finish counting time

end

