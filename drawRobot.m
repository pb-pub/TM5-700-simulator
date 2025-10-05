function [] = drawRobot(th1,th2,th3,th4,th5,th6)
% Draw the robot in a 3D plot based on the provided joint angles.
% Input: Joint angles (th1, th2, th3, th4, th5, th6) in degrees
% Output: None (the function creates a 3D plot)

drawWorkspace(); % Draw the workspace first

% DH Parameters
alpha0 = 0; a0 = 0; d1 = 0.1451;
alpha1 = -pi/2; a1 = 0.329; d2 = 0;
alpha2 = 0; a2 = 0.3115; d3 = 0;
alpha3 = -pi/2; a3 = 0; d4 = -0.1222;
alpha4 = pi/2; a4 = 0; d5 = 0.106;
alpha5 = -pi/2; a5 = 0; d6 = 0.11315;

% Compute the positions of each joint
T01 = mdh(a0, alpha0, d1, deg2rad(th1));
T12 = mdh(a1, alpha1, d2, deg2rad(th2));
T23 = mdh(a2, alpha2, d3, deg2rad(th3));
T34 = mdh(a3, alpha3, d4, deg2rad(th4));
T45 = mdh(a4, alpha4, d5, deg2rad(th5));
T56 = mdh(a5, alpha5, d6, deg2rad(th6));
T02 = T01 * T12;
T03 = T02 * T23;
T04 = T03 * T34;
T05 = T04 * T45;
T06 = T05 * T56;
% Joint positions
x0 = 0; y0 = 0; z0 = 0; %
x1 = T01(1,4); y1 = T01(2,4); z1 = T01(3,4);
x2 = T02(1,4); y2 = T02(2,4); z2 = T02(3,4);
x3 = T03(1,4); y3 = T03(2,4); z3 = T03(3,4);
x4 = T04(1,4); y4 = T04(2,4); z4 = T04(3,4);
x5 = T05(1,4); y5 = T05(2,4); z5 = T05(3,4);
x6 = T06(1,4); y6 = T06(2,4); z6 = T06(3,4);



% Plot the robot links
plot3([x0, x1], [y0, y1], [z0, z1], 'Color', [0.33 0.33 0.33], 'LineWidth', 5); 
plot3([x1, x2], [y1, y2], [z1, z2], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3([x2, x3], [y2, y3], [
    z2, z3], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3([x3, x4], [y3, y4], [z3, z4], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3([x4, x5], [y4, y5], [z4, z5], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3([x5, x6], [y5, y6], [z5, z6], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);

% Plot the joints
plot3(x0, y0, z0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k'); 
plot3(x1, y1, z1, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(x2, y2, z2, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(x3, y3, z3, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');   
plot3(x4, y4, z4, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(x5, y5, z5, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
plot3(x6, y6, z6, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');

hold off;

end