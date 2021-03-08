function output = trajectorycon(x)

    subtractedValues = circshift(x,1) - x - 2;
    c = subtractedValues(2:end);
    c = c(:); % Force to be a column vector
    
    % Adding the constraint that each successive delta x has to be less
    % than double the previous delta x
    for i = 3:length(x)
        
        c = [c; x(i) - x(i-1) - 2*(x(i-1) - x(i-2))];
        
    end

    %ceq = x(1);
    
    output.inequalityConstraints = c;
    %output.equalityConstraints = ceq;

end