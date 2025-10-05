function [] = drawWorkspace()
% Draw the TM5-700 robotic arm's workspace
% Input: None
% Output: None (the function creates a 3D plot)


N_samples = 1e5; % nombre de points réaliste
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

    [x,y,z,~,~,~] = forward(t1,t2,t3,t4,t5,t6);
    X(i) = x;
    Y(i) = y;
    Z(i) = z;

    if mod(i,1000)==0
        fprintf('Progress: %.2f%%\n', (i/N_samples)*100);
    end
end

disp('Workspace points calculation complete.');

% Plot the workspace
figure();
shp = alphaShape(X,Y,Z,10);  
plot(shp, 'FaceColor','cyan','FaceAlpha',0.2,'EdgeColor','none');
hold on;
scatter3(X,Y,Z,3,'r','filled', 'MarkerFaceAlpha', 0.3);

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('TM5-700 Robotic Arm Workspace');
axis equal;
grid on;


end