function [x,y,z,alpha,beta,gamma] = forward(t1,t2,t3,t4,t5,t6)
% Forward kinematics function for a TM5-700 robotic arm
% Input: Joint angles t1 to t6 in degrees
% Output: End-effector position (x, y, z) and orientation (alpha, beta, gamma) in degrees

    % Convert degrees to radians
    t1 = deg2rad(t1);
    t2 = deg2rad(t2);
    t3 = deg2rad(t3);
    t4 = deg2rad(t4);
    t5 = deg2rad(t5);
    t6 = deg2rad(t6);

    % DH Parameters
    alpha0 = 0; a0 = 0; d1 = 0.1451;
    alpha1 = -pi/2; a1 = 0.329; d2 = 0;
    alpha2 = 0; a2 = 0.3115; d3 = 0;
    alpha3 = -pi/2; a3 = 0; d4 = -0.1222;
    alpha4 = pi/2; a4 = 0; d5 = 0.106;
    alpha5 = -pi/2; a5 = 0; d6 = 0.11315;


    % Transformation Matrix
    T01 = mdh(a0, alpha0, d1, t1);
    T12 = mdh(a1, alpha1, d2, t2);
    T23 = mdh(a2, alpha2, d3, t3);
    T34 = mdh(a3, alpha3, d4, t4);
    T45 = mdh(a4, alpha4, d5, t5);
    T56 = mdh(a5, alpha5, d6, t6);

    T = T01 * T12 * T23 * T34 * T45 * T56;

    
    % Extract position
    x = T(1,4);
    y = T(2,4);
    z = T(3,4);
    % Extract orientation (ZYX Euler angles)
    beta = atan2(-T(3,1), sqrt(T(1,1)^2 + T(2,1)^2));
    alpha = atan2(T(2,1), T(1,1));
    gamma = atan2(T(3,2), T(3,3));


    % Convert radians to degrees
    alpha = rad2deg(alpha);
    beta = rad2deg(beta);
    gamma = rad2deg(gamma);

end