%% Main script for the kinematics of the TM5-700 robotic arm
clear; close all; clc;


disp('Select an option:');
disp('1. Forward Kinematics');
disp('2. Inverse Kinematics');
disp('3. Visualize Robot Workspace');
choice = input('Enter your choice (1/2/3): ');

switch choice
        case 1
                disp('You selected forward kinematics.');
                joint_angles = zeros(1,6);
                for i = 1:6
                        prompt = sprintf('Enter joint angle t%d (in degrees): ', i);
                        joint_angles(i) = input(prompt);
                end

                % Compute forward kinematics
                [x, y, z, alpha, beta, gamma] = forward(joint_angles(1), joint_angles(2), joint_angles(3), ...
                                                       joint_angles(4), joint_angles(5), joint_angles(6), dh_parameters());

                fprintf('End-Effector Position: (%f, %f, %f)\n', x, y, z);
                fprintf('End-Effector Orientation (ZYX Euler angles): (%f°, %f°, %f°)\n', alpha, beta, gamma);

        case 2
                disp('You selected inverse kinematics.');

                x = input('Enter desired end-effector position x (in meters): ');
                y = input('Enter desired end-effector position y (in meters): ');
                z = input('Enter desired end-effector position z (in meters): ');
                alpha = input('Enter desired end-effector orientation alpha (in degrees): ');
                beta = input('Enter desired end-effector orientation beta (in degrees): ');
                gamma = input('Enter desired end-effector orientation gamma (in degrees): ');

                % Compute inverse kinematics
                [list_thetas, success] = inverse(x, y, z, alpha, beta, gamma, dh_parameters());

                switch success
                    case 1
                        fprintf('Inverse kinematics solutions found for the given target [%f, %f, %f, %f, %f, %f]:\n', x, y, z, alpha, beta, gamma);
                        for i = 1:size(list_thetas, 1)
                            [x, y, z, alpha, beta, gamma] = forward(list_thetas(i,1), list_thetas(i,2), list_thetas(i,3), ...
                                list_thetas(i,4), list_thetas(i,5), list_thetas(i,6), dh_parameters());
                            fprintf('Solution %d: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] with position : (%.4f, %.4f, %.4f) and orientation : (%.2f°, %.2f°, %.2f°)\n', i, list_thetas(i, 1), list_thetas(i, 2), list_thetas(i, 3), list_thetas(i, 4), list_thetas(i, 5), list_thetas(i, 6), x, y, z, alpha, beta, gamma);
                        end
                    case 0
                        disp('No valid inverse kinematics solutions found for the given target.');
                    case -1
                        disp('Target position is out of reach.');
                end

        case 3
                disp('You selected robot workspace visualization.');
                joint_angles = [0,30,150,0,0,0];
                for i = 1:6
                        prompt = sprintf('Enter joint angle t%d (in degrees): ', i);
                        joint_angles(i) = input(prompt);
                end
                drawRobot(joint_angles(1), joint_angles(2), joint_angles(3), ...
                          joint_angles(4), joint_angles(5), joint_angles(6), dh_parameters());
                hold off;

        otherwise
                disp('Invalid selection.');
end
