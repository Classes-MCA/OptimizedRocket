clear; close all;

velocity = 0:100:2e3; % m/s
height = 0:100:250e3; % meters

[Velocity, Height] = meshgrid(velocity,height);

for i = 1:length(velocity)
    
    for j = 1:length(height)
        
        drag(j,i) = getDrag2(velocity(i),height(j));
        
    end
    
end

figure()
surf(Height./1000, Velocity, drag, 'MeshStyle','None')
title('Rocket Drag')
xlabel('Height (km)')
ylabel('Velocity (m/s)')
zlabel('Drag (N)')

