clear; close all;

% Purpose: to show a quick introductory example of how we might go about
% doing this project

% First things first, we'll want to use "Object-oriented Programming",
% which means that we create MATLAB objects that are then editable. This
% helps us to avoid creating all kinds of large arrays and having too many
% numbers that clutter up our main files. I've defined a MATLAB object
% called "Rocket", and the code for this object is in the 'src/' folder.

% To define a Rocket object, type:
bill = Rocket; % The name is arbitrary, but we'll just call it 'bill' for
               % now to help keep things a little light-hearted
               
% You can copy-paste that line of code into your command window, and remove
% the semicolon to see all of the different variables (properties) within 
% the Rocket object. These include things like 'Location', 'Thrust',
% 'Fuel', and 'Velocity'. These are editable like so:
bill.Velocity = [0, 5];
bill.Fuel = 600;
bill.Mass = 4500;

% If you run those lines and then type 'bill' into the command line then
% you'll see that the values have changed.

% One of the ways in which an object is different from a structured
% variable (structs) is that an object can also contain functions. For
% example:
bill.updateLocation;
bill.updateFuel;

% In the above examples, the 'updateLocation' function takes the current
% velocity, multiplied by the step size, and generates the new location of
% the rocket in x,y,z coordinates. The 'updateFuel' function takes the fuel
% consumption rate, multiplied by the step size, to return the remaining
% fuel in the tanks. These could be run in a loop to update the rocket's
% position and fuel:

maxSteps = 10;
trajectory = zeros(maxSteps,2);
for i = 1:maxSteps
    bill.Velocity = [0, bill.Velocity(2) + 10]; % Speed up after each step
    bill.updateLocation;
    bill.updateFuel;
    trajectory(i,:) = bill.Location; % storing the trajectory
    disp(['Step Number ',num2str(i),' Velocity = [',num2str(bill.Velocity),...
          '] Location = [',num2str(bill.Location),']'])
end

figure()
plot(trajectory(:,2))
xlabel('Step Number')
ylabel('Upward Velocity (m/s)')
title('Sample Rocket Velocity Plot')
grid on