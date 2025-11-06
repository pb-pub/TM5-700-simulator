function [list_thetas, success] = inverse(x, y, z, alpha, beta, gamma, dh_params)
% Inverse kinematics function for a TM5-700 robotic arm.
% This function calculates the joint angles for the robotic arm to achieve the desired end-effector position and orientation.
% Input: Desired end-effector position (x, y, z), orientation (alpha, beta, gamma) in degrees, and dh_params (struct)
% Output: Joint angles (list_thetas) in degrees and success flag (success)

list_thetas = [];


% Compute the transformation matrix from the desired end-effector position and orientation
R = [[cosd(alpha), -sind(alpha), 0]; [sind(alpha), cosd(alpha), 0]; [0, 0, 1]]...
    * [[cosd(beta), 0, sind(beta)]; [0, 1, 0]; [-sind(beta), 0, cosd(beta)]]...
    * [[1, 0, 0]; [0, cosd(gamma), -sind(gamma)]; [0, sind(gamma), cosd(gamma)]];


% Solve analytically for theta_5, theta_1, theta_6
A = (x + dh_params.d(6) * R(1,3))^2 + (y + dh_params.d(6) * R(2,3))^2;
B = 2 * dh_params.d(4) * ( (x + dh_params.d(6) * R(1,3)) * R(1,3) + (y + dh_params.d(6) * R(2,3)) * R(2,3) );
C = dh_params.d(4)^2 * (R(1,3)^2 + R(2,3)^2) - (y * R(1,3) - x * R(2,3))^2;

discriminant = B^2 - 4 * A * C;
if discriminant < 0
    list_thetas = [];
    success = -1;
    warning('No valid solution');
    return;
elseif discriminant == 0
    % Here there is 2 solutions for theta_5 so we add 2 nodes to the tree
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(acos(-B / (2 * A))), inf]];
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(-acos(-B / (2 * A))), inf]];

else
    % Here there is 4 solutions for theta_5 so we add 4 nodes to the tree
    sqrt_disc = sqrt(discriminant);
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(acos((-B + sqrt_disc) / (2 * A))), inf]];
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(acos((-B - sqrt_disc) / (2 * A))), inf]];
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(-acos((-B + sqrt_disc) / (2 * A))), inf]];
    list_thetas = [list_thetas; [inf, inf, inf, inf, rad2deg(-acos((-B - sqrt_disc) / (2 * A))), inf]];

end

% computing of thetas 1
temp_list_thetas = [];

r = sqrt(x^2 + y^2);
base_th1 = atan2d(-y, x);
for i = 1:size(list_thetas, 1)
    theta_5 = list_thetas(i, 5);
    if sind(theta_5) == 0
        continue; % Skip this iteration if sin(theta_5) is zero to avoid division by zero
    end

    new = list_thetas(i, :);
    new(1) = base_th1 + atan2d(sqrt(1 - ((-dh_params.d(4) - cosd(theta_5)*dh_params.d(6))/r)^2), (-dh_params.d(4) - cosd(theta_5)*dh_params.d(6))/r);
    temp_list_thetas = [temp_list_thetas; new];

    new(1) = base_th1 + atan2d(-sqrt(1 - ((-dh_params.d(4) - cosd(theta_5)*dh_params.d(6))/r)^2), (-dh_params.d(4) - cosd(theta_5)*dh_params.d(6))/r);
    temp_list_thetas = [temp_list_thetas; new];
end
list_thetas = temp_list_thetas;
temp_list_thetas = [];


% Computing of thetas 6
for i = 1:size(list_thetas, 1)
    th1 = list_thetas(i, 1);
    th5 = list_thetas(i, 5);

    list_thetas(i, 6) = atan2d( ( cosd(th1)*R(2,2) - sind(th1)*R(1, 2) ) / sind(th5), ( sind(th1)*R(1, 1) - cosd(th1)*R(2,1) ) / sind(th5) );
end


% Numerics estimation of thetas 4 and cascadingly thetas 2 and 3

h = waitbar(0, 'Computing Inverse Kinematics...');

for i = 1:size(list_thetas, 1)
    th1 = list_thetas(i, 1);
    th5 = list_thetas(i, 5);
    th6 = list_thetas(i, 6);

    c1 = cosd(th1);
    s1 = sind(th1);
    c5 = cosd(th5);
    s5 = sind(th5);
    c6 = cosd(th6);
    s6 = sind(th6);

    theta4_range = linspace(dh_params.th_limits(4, 1), dh_params.th_limits(4, 2), 10000);
    min_error = [inf,inf];
    best_th4 = [NaN, NaN];
    list_theta1 = list_thetas(i, :);
    list_theta2 = list_thetas(i, :);
    for idxth4 = 1:length(theta4_range)
        th4 = theta4_range(idxth4);

        c4 = cosd(th4);
        s4 = sind(th4);
        K1 = c1*x + s1*y;
        K2 = z - dh_params.d(1);
        K5 = dh_params.a(3+1) - c4*s5*dh_params.d(6) + s4*dh_params.d(5);
        K6 = -s4*s5*dh_params.d(6) - c4*dh_params.d(5);

        r3 = sqrt(K5^2 + K6^2);
        try 
            th3_1 = atan2d(sqrt(1 - ( (K1^2 + K2^2 - K5^2 - K6^2 - dh_params.a(2+1)^2     ) / (2 * dh_params.a(2+1) * r3) )^2), ...
            (K1^2 + K2^2 - K5^2 - K6^2 - dh_params.a(2+1)^2) / (2 * dh_params.a(2+1) * r3));

            c3_1 = cosd(th3_1);
            s3_1 = sind(th3_1);


            K3_1 = dh_params.a(2+1) + dh_params.a(3+1)*c3_1 + dh_params.d(5)*sind(th3_1 + th4) - dh_params.d(6)*s5*cosd(th3_1 + th4);
            K4_1 = dh_params.a(2+1)*s3_1 + dh_params.d(5)*cosd(th3_1 + th4) - dh_params.d(6)*s5*sind(th3_1 + th4);

            th2_1 = atan2d(K2*K3_1 - K1*K4_1, K1*K3_1 + K2*K4_1);

            [test_x, test_y, test_z, test_alpha, test_beta, test_gamma] = forward(th1, th2_1, th3_1, th4, th5, th6, dh_params);

            error = sqrt((test_x - x)^2 + (test_y - y)^2 + (test_z - z)^2 + ((test_alpha - alpha)/180)^2 + ((test_beta - beta)/180)^2 + ((test_gamma - gamma)/180)^2);

            if error < min_error(1)
                min_error(1) = error;
                best_th4(1) = th4;
                list_theta1(2) = th2_1;
                list_theta1(3) = th3_1;
            end

        catch ME
            % warning(ME.message);
        end
        
        try 
            th3_2 = atan2d(-sqrt(1 - ( (K1^2 + K2^2 - K5^2 - K6^2 - dh_params.a(2+1)^2     ) / (2 * dh_params.a(2+1) * r3) )^2), ...
                (K1^2 + K2^2 - K5^2 - K6^2 - dh_params.a(2+1)^2) / (2 * dh_params.a(2+1) * r3));

            c3_2 = cosd(th3_2);
            s3_2 = sind(th3_2);

            K3_2 = dh_params.a(2+1) + dh_params.a(3+1)*c3_2 + dh_params.d(5)*sind(th3_2 + th4) - dh_params.d(6)*s5*cosd(th3_2 + th4);
            K4_2 = dh_params.a(2+1)*s3_2 + dh_params.d(5)*cosd(th3_2 + th4) - dh_params.d(6)*s5*sind(th3_2 + th4);

            
            th2_2 = atan2d(K2*K3_2 - K1*K4_2, K1*K3_2 + K2*K4_2);

            [test_x, test_y, test_z, test_alpha, test_beta, test_gamma] = forward(th1, th2_2, th3_2, th4, th5, th6, dh_params);

            error = sqrt((test_x - x)^2 + (test_y - y)^2 + (test_z - z)^2 + ((test_alpha - alpha)/180)^2 + ((test_beta - beta)/180)^2 + ((test_gamma - gamma)/180)^2);
            
            if error < min_error(2)
                min_error(2) = error;
                best_th4(2) = th4;
                list_theta2(2) = th2_2;
                list_theta2(3) = th3_2;
            end
        catch ME
            % warning(ME.message);
        end

        waitbar((idxth4 / length(theta4_range) + i*length(theta4_range)) / (length(theta4_range) * size(list_thetas, 1)), h);

    end
    if ~isnan(best_th4(1))
        list_theta1(4) = best_th4(1);
        temp_list_thetas = [temp_list_thetas; list_theta1];
    end
    if ~isnan(best_th4(2))
        list_theta2(4) = best_th4(2);
        temp_list_thetas = [temp_list_thetas; list_theta2];
    end
end
list_thetas = temp_list_thetas;

if ~isempty(list_thetas)
    success = 1;
else
    success = 0;
    warning('No valid solutions found');

end

end