function output = trajectoryInequalityCon(x)

    subtractedValues = circshift(x,1) - x - 2;
    c = subtractedValues(2:end);
    c = c(:); % Force to be a column vector
    
    % Adding the constraint that each successive delta x has to be less
    % than double the previous delta x
    % Spacing out the x-values again
%     for i = 3:length(x)
%         
%         c = [c; (x(i) - x(i-1) - 3*(x(i-1) - x(i-2)))];
%         
%     end

    % Adding the constraint that the angle should not be too large between
    % iterations
%     subtractedDerivatives = circshift(diff(x),1) - diff(x) - 10;
%     subtractedDerivatives = subtractedDerivatives(:);
%     c = [c; subtractedDerivatives(2:end)];

    % Setting the endpoint derivative
%     deltaY = 840;
%     deriv = (42e3 - (42e3 - deltaY)) / (x(end-1) - x(end));
%     c = [c;abs(deriv) - 100];
    
    output.inequalityConstraints = c;

end
