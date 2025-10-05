function [] = drawWorkspace()
% Draw the TM5-700 robotic arm's workspace

    % Define the range of joint angles (in degrees)
    t1_range = -270:30:270;
    t2_range = -180:30:180;
    t3_range = -155:30:155;
    t4_range = -180:30:180;
    t5_range = -180:30:180;
    t6_range = -270:30:270;

    % Initialize arrays to hold end-effector positions
    X = [];
    Y = [];
    Z = [];

    % Loop through all combinations of joint angles
    for t1 = t1_range
        for t2 = t2_range
            for t3 = t3_range
                for t4 = t4_range
                    for t5 = t5_range
                        for t6 = t6_range
                            [x, y, z, ~, ~, ~] = forward(t1, t2, t3, t4, t5, t6);
                            X = [X; x];
                            Y = [Y; y];
                            Z = [Z; z];
                        end
                    end
                end
            end
        end
    end

    % Plot the workspace
    figure;
    scatter3(X, Y, Z, 1, 'filled');
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('TM5-700 Robotic Arm Workspace');
    axis equal;
    grid on;


end