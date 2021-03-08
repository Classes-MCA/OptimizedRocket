function output = trajectoryEqualityCon(x)

    % Setting the endpoint
    ceq = x(end) - 10e3;
    
    output.equalityConstraints = ceq;
    

end