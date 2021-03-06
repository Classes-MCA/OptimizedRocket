function finalMass = objective(x)

    targetX = x(end); % meters
    targetY = 42e3; % meters
    
    deltaY = targetY / length(x); % meters
    
    fuelExitVelocity = 2000; % meters/second
    
    g = 9.8; % m/s^2
    a = 3*g; % m/s^2
    
    mass = zeros(1,length(x));
    mass(1) = 1e4; % initial mass
    
    velocity = zeros(1,length(x));
    velocity(1) = 10;
    
    time = zeros(1,length(x));

    % Iterate over each x-position
    for i = 2:length(x)
        
        % Getting initial conditions going for this round
        deltaX = x(i) - x(i-1);
        height = (i-1) * deltaY;
        
        % Calculate the tilt angle
        theta = atan(deltaX/deltaY);
        
        % Calculate the drag
        D = 0; % no drag for now
        
        % calculate the thrust
        Tx = (mass(i-1) * a + D) * sin(theta);
        Ty = (mass(i-1) * a + D) * cos(theta) + mass(i-1) * g;
        T = sqrt(Tx^2 + Ty^2);
        
        % calculate the change in mass
        deltaTime = sqrt(deltaX^2 + deltaY^2) / velocity(i-1);
        deltaMass = T / fuelExitVelocity * deltaTime;
        
        % Update some values for the next round
        velocity(i) = velocity(i-1) + a * deltaTime;
        mass(i) = mass(i-1) - deltaMass;
        time(i) = time(i-1) + deltaTime;
        
    end
    
    finalMass = mass(end);
    y = 0:deltaY:targetY - deltaY;
    
    figure()
    subplot(2,1,1)
    plot(x./1000,y./1000)
    title("Trajectory")
    xlabel("X (km)")
    ylabel("Y (km)")
    
    subplot(2,1,2)
    plot(time,mass)
    title("Mass")
    xlabel("Time (s)")
    ylabel("Rocket Mass (kg)")

end