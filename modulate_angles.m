function thetas_modulated = modulate_angles(thetas, dh_parameters)
% MODULATE_ANGLES Modulates joint angles to have all possibilities within the range of the robot's joint limits.
% Input:
%   thetas - Matrix of joint angles (in degrees), where each row
%            represents a set of joint angles.
%   dh_parameters - Struct containing the Denavit-Hartenberg parameters,
%                   including joint limits.
% Output:
%   thetas_modulated - Matrix of modulated joint angles (in degrees).  


thetas_modulated = [];
for i = 1:size(thetas, 1)
    current_thetas = thetas(i, :);
    modulated_set = current_thetas; % Start with the original set

    for j = 1:length(current_thetas)
        joint_min = dh_parameters.th_limits(j, 1);
        joint_max = dh_parameters.th_limits(j, 2);

        % Generate additional angles by adding/subtracting multiples of 360 degrees
        k = 1;
        while true
            new_angle_pos = current_thetas(j) + k * 360;
            if new_angle_pos <= joint_max
                new_set = current_thetas;
                new_set(j) = new_angle_pos;
                modulated_set = [modulated_set; new_set];
            else
                break;
            end
            k = k + 1;
        end

        k = 1;
        while true
            new_angle_neg = current_thetas(j) - k * 360;
            if new_angle_neg >= joint_min
                new_set = current_thetas;
                new_set(j) = new_angle_neg;
                modulated_set = [modulated_set; new_set];
            else
                break;
            end
            k = k + 1;
        end
    end

    % Append all modulated sets for the current row to the output
    thetas_modulated = [thetas_modulated; modulated_set];
end

end