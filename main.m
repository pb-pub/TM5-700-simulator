%% Main script for the kinematics project

clear; clc; close all;


%% Part 1: Forward Kinematics
disp('--- Forward Kinematics ---');

% Define joint angles (in degrees)
joint_angles = [30, 45, 60, 90, 45, 30]; % Exemple


% Compute forward kinematics
[x, y, z, alpha, beta, gamma] = forward(joint_angles(1), joint_angles(2), joint_angles(3), ...
                                       joint_angles(4), joint_angles(5), joint_angles(6));
% Display results
fprintf('End-Effector Position: (%.4f, %.4f, %.4f)\n', x, y, z);
fprintf('End-Effector Orientation (ZYX Euler angles): (%.2f°, %.2f°, %.2f°)\n', alpha, beta, gamma);

%% Part 2: Inverse Kinematics
disp('--- Inverse Kinematics ---');
% Define desired end-effector position and orientation
desired_position = [0.5, 0.2, 0.3]; %
desired_orientation = [30, 45, 60]; % (alpha, beta, gamma) in degrees
% Compute inverse kinematics
list_thetas = inverse(desired_position(1), desired_position(2), desired_position(3), ...
                      desired_orientation(1), desired_orientation(2), desired_orientation(3));


% Display results
fprintf('Computed Joint Angles: (%.2f°, %.2f°, %.2f°, %.2f°, %.2f°, %.2f°)\n', ...
        list_thetas(1), list_thetas(2), list_thetas(3), list_thetas(4), list_thetas(5), list_thetas(6));

%% Part 3: Workspace Visualization

disp('--- Workspace Visualization ---');
figure;
drawWorkspace();
disp('Workspace visualization complete.');