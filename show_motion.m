function show_motion(thetas, dh_parameters, ax)
% SHOW_MOTION Visualizes the end_effector path in 3D based on the provided joint angles over time
% Input:
%   thetas - A matrix where each row represents joint angles at a specific time step
%   dh_parameters - Struct containing the Denavit-Hartenberg parameters of the robot
%   ax - (optional) Axes handle to draw the robot and path on. If omitted, uses gca.


if (nargin < 3)
    ax = gca;
end

num_steps = size(thetas, 1);
end_effector_path = zeros(num_steps, 3); % Store only X, Y, Z
for step = 1:num_steps
    [x,y,z,~,~,~] = forward(thetas(step,1), thetas(step,2), thetas(step,3), ...
        thetas(step,4), thetas(step,5), thetas(step,6), dh_parameters);
    end_effector_path(step, :) = [x, y, z];
end   


drawRobot(thetas(1,1), thetas(1,2), thetas(1,3), ...
          thetas(1,4), thetas(1,5), thetas(1,6), dh_parameters, ax);


% Ensure we're plotting on the provided axes
hold(ax, 'on');
plot3(ax, end_effector_path(:,1), end_effector_path(:,2), end_effector_path(:,3), 'r-', 'LineWidth', 2);
xlabel(ax, 'X (m)');
ylabel(ax, 'Y (m)');
zlabel(ax, 'Z (m)');
view(ax, 3);
end