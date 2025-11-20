function make_gif(thetas, dh_parameters, filename)
% MAKE_GIF Creates a GIF of the robot's motion
% Input:
%   thetas - A matrix where each row represents joint angles at a specific time step
%   dh_parameters - Struct containing the Denavit-Hartenberg parameters of the robot
%   filename - The name of the GIF file to create

if nargin < 3
    filename = 'robot_motion.gif';
end

h = figure;
ax = gca;
axis tight;  % ensure axes limits fit data
axis manual; % freeze axis limits so getframe() is consistent
xlim(ax, [-0.8 0.8]); % Set X limits based on expected robot workspace
ylim(ax, [-0.8 0.8]); % Set Y limits based on expected robot workspace
zlim(ax, [-0.8 0.8]); % Set Z limits based on expected robot workspace
drawnow;

pat = points_and_times();

% Draw workspace once
drawWorkspace(dh_parameters, ax);
hold(ax, 'on');

num_steps = size(thetas, 1);
end_effector_path = zeros(num_steps, 3); % Store only X, Y, Z

for step = 1:num_steps
    [x,y,z,~,~,~] = forward(thetas(step,1), thetas(step,2), thetas(step,3), ...
        thetas(step,4), thetas(step,5), thetas(step,6), dh_parameters);
    end_effector_path(step, :) = [x, y, z];
end

% Extract DH parameters for repeated use
a0 = dh_parameters.a(0+1); alpha0 = dh_parameters.alpha(0+1); d1 = dh_parameters.d(1);
a1 = dh_parameters.a(1+1); alpha1 = dh_parameters.alpha(1+1); d2 = dh_parameters.d(2);
a2 = dh_parameters.a(2+1); alpha2 = dh_parameters.alpha(2+1); d3 = dh_parameters.d(3);
a3 = dh_parameters.a(3+1); alpha3 = dh_parameters.alpha(3+1); d4 = dh_parameters.d(4);
a4 = dh_parameters.a(4+1); alpha4 = dh_parameters.alpha(4+1); d5 = dh_parameters.d(5);
a5 = dh_parameters.a(5+1); alpha5 = dh_parameters.alpha(5+1); d6 = dh_parameters.d(6);

for step = 1:num_steps
    % Clear only the robot plot elements, not the whole axis
    if step > 1
        delete(robot_plots);
    end

    th1 = thetas(step,1); th2 = thetas(step,2); th3 = thetas(step,3);
    th4 = thetas(step,4); th5 = thetas(step,5); th6 = thetas(step,6);

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
    x0=0; y0=0; z0=0;
    x1=T01(1,4); y1=T01(2,4); z1=T01(3,4);
    x2=T02(1,4); y2=T02(2,4); z2=T02(3,4);
    x3=T03(1,4); y3=T03(2,4); z3=T03(3,4);
    x4=T04(1,4); y4=T04(2,4); z4=T04(3,4);
    x5=T05(1,4); y5=T05(2,4); z5=T05(3,4);
    x6=T06(1,4); y6=T06(2,4); z6=T06(3,4);

    % Plot the robot links and joints
    robot_plots = gobjects(1, 13);
    robot_plots(1) = plot3(ax, [x0, x1], [y0, y1], [z0, z1], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    robot_plots(2) = plot3(ax, [x1, x2], [y1, y2], [z1, z2], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    robot_plots(3) = plot3(ax, [x2, x3], [y2, y3], [z2, z3], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    robot_plots(4) = plot3(ax, [x3, x4], [y3, y4], [z3, z4], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    robot_plots(5) = plot3(ax, [x4, x5], [y4, y5], [z4, z5], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    robot_plots(6) = plot3(ax, [x5, x6], [y5, y6], [z5, z6], 'Color', [0.33 0.33 0.33], 'LineWidth', 5);
    
    jointColors = {[1 0 0], [0 1 0], [0 0 1], [1 0.5 0], [0.5 0 0.5], [0 0.7 0.7], [0.7 0.7 0]};
    robot_plots(7) = plot3(ax, x0, y0, z0, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{1}, 'MarkerEdgeColor', jointColors{1});
    robot_plots(8) = plot3(ax, x1, y1, z1, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{2}, 'MarkerEdgeColor', jointColors{2});
    robot_plots(9) = plot3(ax, x2, y2, z2, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{3}, 'MarkerEdgeColor', jointColors{3});
    robot_plots(10) = plot3(ax, x3, y3, z3, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{4}, 'MarkerEdgeColor', jointColors{4});
    robot_plots(11) = plot3(ax, x4, y4, z4, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{5}, 'MarkerEdgeColor', jointColors{5});
    robot_plots(12) = plot3(ax, x5, y5, z5, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{6}, 'MarkerEdgeColor', jointColors{6});
    robot_plots(13) = plot3(ax, x6, y6, z6, 'o', 'MarkerSize', 8, 'MarkerFaceColor', jointColors{7}, 'MarkerEdgeColor', jointColors{7});

    % Plot the end effector path
    if step > 1
        delete(path_plot);
    end
    path_plot = plot3(ax, end_effector_path(1:step,1), end_effector_path(1:step,2), end_effector_path(1:step,3), 'r-', 'LineWidth', 2, "DisplayName", "End Effector Path");
    plot3(ax, pat.A(1,4), pat.A(2,4), pat.A(3,4), 'go', 'MarkerSize', 8, 'DisplayName', 'Point A');
    plot3(ax, pat.B(1,4), pat.B(2,4), pat.B(3,4), 'bo', 'MarkerSize', 8, 'DisplayName', 'Point B');
    plot3(ax, pat.C(1,4), pat.C(2,4), pat.C(3,4), 'mo', 'MarkerSize', 8, 'DisplayName', 'Point C');
    
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    view(3);
    drawnow;

    % Capture the plot as an image
    frame = getframe(h);
    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);

    % Write to the GIF File
    if step == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf, 'DelayTime', 0.01);
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append', 'DelayTime', 0.01);
    end
end

hold(ax, 'off');
end
