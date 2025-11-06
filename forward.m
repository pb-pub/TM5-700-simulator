function [x,y,z,alpha,beta,gamma] = forward(t1,t2,t3,t4,t5,t6, dh_params)
% Forward kinematics function for a TM5-700 robotic arm
% Input: Joint angles t1 to t6 in degrees and dh_params (struct)
% Output: End-effector position (x, y, z) and orientation (alpha, beta, gamma) in degrees


    % Convert degrees to radians
    t1 = deg2rad(t1);
    t2 = deg2rad(t2);
    t3 = deg2rad(t3);
    t4 = deg2rad(t4);
    t5 = deg2rad(t5);
    t6 = deg2rad(t6);

    % Extract DH parameters
    % I wrote +1 here to show that the indexing starts from 1 in MATLAB
    % while the DH parameters alpha and a are usually indexed from 0 
    a0 = dh_params.a(0+1); alpha0 = dh_params.alpha(0+1); d1 = dh_params.d(1);
    a1 = dh_params.a(1+1); alpha1 = dh_params.alpha(1+1); d2 = dh_params.d(2);
    a2 = dh_params.a(2+1); alpha2 = dh_params.alpha(2+1); d3 = dh_params.d(3);
    a3 = dh_params.a(3+1); alpha3 = dh_params.alpha(3+1); d4 = dh_params.d(4);
    a4 = dh_params.a(4+1); alpha4 = dh_params.alpha(4+1); d5 = dh_params.d(5);
    a5 = dh_params.a(5+1); alpha5 = dh_params.alpha(5+1); d6 = dh_params.d(6);

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
    beta = atan2d(-T(3,1), sqrt(T(1,1)^2 + T(2,1)^2));
    alpha = atan2d(T(2,1), T(1,1));
    gamma = atan2d(T(3,2), T(3,3));


end