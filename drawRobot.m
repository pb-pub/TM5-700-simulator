function [] = drawRobot(th1,th2,th3,th4,th5,th6, dh_params, ax)
% Draw the robot in a 3D plot based on the provided joint angles.
% Input: Joint angles (th1, th2, th3, th4, th5, th6) in degrees and dh_params (struct)
% Output: None (the function creates a 3D plot)

if nargin < 8
    figure;
    ax = gca;
end

axes(ax);    
hold(ax, 'on');
drawWorkspace(dh_params, ax); % Draw the workspace first

 % Extract DH parameters
% I wrote +1 here to show that the indexing starts from 1 in MATLAB
% while the DH parameters alpha and a are usually indexed from 0 
a0 = dh_params.a(0+1); alpha0 = dh_params.alpha(0+1); d1 = dh_params.d(1);
a1 = dh_params.a(1+1); alpha1 = dh_params.alpha(1+1); d2 = dh_params.d(2);
a2 = dh_params.a(2+1); alpha2 = dh_params.alpha(2+1); d3 = dh_params.d(3);
a3 = dh_params.a(3+1); alpha3 = dh_params.alpha(3+1); d4 = dh_params.d(4);
a4 = dh_params.a(4+1); alpha4 = dh_params.alpha(4+1); d5 = dh_params.d(5);
a5 = dh_params.a(5+1); alpha5 = dh_params.alpha(5+1); d6 = dh_params.d(6);

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
plot3(ax, [x0, x1], [y0, y1], [z0, z1], 'Color', [0.33 0.33 0.33], 'LineWidth', 5); 
plot3(ax, [x1, x2], [y1, y2], [z1, z2], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3(ax, [x2, x3], [y2, y3], [
    z2, z3], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3(ax, [x3, x4], [y3, y4], [z3, z4], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3(ax, [x4, x5], [y4, y5], [z4, z5], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
plot3(ax, [x5, x6], [y5, y6], [z5, z6], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);

% Plot the joints
jointColors = {[1 0 0], [0 1 0], [0 0 1], [1 0.5 0], [0.5 0 0.5], [0 0.7 0.7], [0.7 0.7 0]}; % red, green, blue, orange, purple, teal, yellow
jointHandles = gobjects(1,7);
jointHandles(1) = plot3(ax, x0, y0, z0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{1}, 'MarkerEdgeColor', jointColors{1});
jointHandles(2) = plot3(ax, x1, y1, z1, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{2}, 'MarkerEdgeColor', jointColors{2});
jointHandles(3) = plot3(ax, x2, y2, z2, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{3}, 'MarkerEdgeColor', jointColors{3});
jointHandles(4) = plot3(ax, x3, y3, z3, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{4}, 'MarkerEdgeColor', jointColors{4});
jointHandles(5) = plot3(ax, x4, y4, z4, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{5}, 'MarkerEdgeColor', jointColors{5});
jointHandles(6) = plot3(ax, x5, y5, z5, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{6}, 'MarkerEdgeColor', jointColors{6});
jointHandles(7) = plot3(ax, x6, y6, z6, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{7}, 'MarkerEdgeColor', jointColors{7});

legend(jointHandles, {'Joint 0 (Base)', 'Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6 (End Effector)'}, 'Location', 'bestoutside');

hold(ax, 'off');

end