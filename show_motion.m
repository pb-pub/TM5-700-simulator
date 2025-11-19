function show_motion(thetas, dh_parameters)
% SHOW_MOTION Visualizes the end_effector path in 3D based on the provided joint angles over time
% Input:
%   thetas - A matrix where each row represents joint angles at a specific time step
%   dh_parameters - Struct containing the Denavit-Hartenberg parameters of the robot


num_steps = size(thetas, 1);
end_effector_path = zeros(num_steps, 3); % Store only X, Y, Z
for step = 1:num_steps
    [x,y,z,~,~,~] = forward(thetas(step,1), thetas(step,2), thetas(step,3), ...
        thetas(step,4), thetas(step,5), thetas(step,6), dh_parameters);
    end_effector_path(step, :) = [x, y, z];
end   


drawRobot(thetas(1,1), thetas(1,2), thetas(1,3), ...
          thetas(1,4), thetas(1,5), thetas(1,6), dh_parameters);


hold on;
plot3(end_effector_path(:,1), end_effector_path(:,2), end_effector_path(:,3), 'r-', 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');           
view(3);
end