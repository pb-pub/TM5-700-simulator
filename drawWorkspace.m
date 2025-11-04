function [] = drawWorkspace(dh_params,ax)
% Draw the TM5-700 robotic arm's workspace
% Input: dh_params (struct)
% Output: None (the function creates a 3D plot)

if nargin < 2
    figure; ax = gca;
end

axes(ax);    
hold(ax, 'on');

N_samples = 1e4; % nombre de points réaliste
X = zeros(N_samples,1);
Y = zeros(N_samples,1);
Z = zeros(N_samples,1);

disp('Calculating workspace points...');
for i = 1:N_samples
    t1 = randi([-270, 270]);
    t2 = randi([-180, 180]);
    t3 = randi([-155, 155]);
    t4 = randi([-180, 180]);
    t5 = randi([-180, 180]);
    t6 = randi([-270, 270]);

    [x,y,z,~,~,~] = forward(t1,t2,t3,t4,t5,t6, dh_params);
    X(i) = x;
    Y(i) = y;
    Z(i) = z;

    if mod(i,1000)==0
        fprintf('Progress: %.2f%%\n', (i/N_samples)*100);
    end
end

disp('Workspace points calculation complete.');

% Plot the workspace
shp = alphaShape(X,Y,Z,10);
[F,V] = boundaryFacets(shp);
patch(ax, 'Faces', F, 'Vertices', V, ...
    'FaceColor','cyan', 'FaceAlpha',0.2, 'EdgeColor','none');

scatter3(ax, X,Y,Z, 3,'r','filled', 'MarkerFaceAlpha', 0.1);

xlabel(ax,'X (m)');
ylabel(ax,'Y (m)');
zlabel(ax,'Z (m)');
title(ax,'TM5-700 Robotic Arm Workspace');
axis(ax,'equal');
grid(ax,'on');



end