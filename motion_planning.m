function thetas =  motion_planning()

    pat = points_and_times();
    dh = dh_parameters();
    disp('Points and Times:');
    disp(pat);
    
    [x,y,z, alpha, beta, gamma] = deal( pat.A(1,4), pat.A(2,4), pat.A(3,4), ...
                                    atan2d(pat.A(3,2), pat.A(3,3)), ...
                                    atan2d(-pat.A(3,1), sqrt(pat.A(3,2)^2 + pat.A(3,3)^2)), ...
                                    atan2d(pat.A(2,1), pat.A(1,1)) );

    thAs = inverse(x, y, z, alpha, beta, gamma, dh);
    
    [x,y,z, alpha, beta, gamma] = deal( pat.B(1,4), pat.B(2,4), pat.B(3,4), ...
                                    atan2d(pat.B(3,2), pat.B(3,3)), ...
                                    atan2d(-pat.B(3,1), sqrt(pat.B(3,2)^2 + pat.B(3,3)^2)), ...
                                    atan2d(pat.B(2,1), pat.B(1,1)) );
    thBs = inverse(x, y, z, alpha, beta, gamma, dh);

    [x,y,z, alpha, beta, gamma] = deal( pat.C(1,4), pat.C(2,4), pat.C(3,4), ...
                                    atan2d(pat.C(3,2), pat.C(3,3)), ...
                                    atan2d(-pat.C(3,1), sqrt(pat.C(3,2)^2 + pat.C(3,3)^2)), ...
                                    atan2d(pat.C(2,1), pat.C(1,1)) );
    thCs = inverse(x, y, z, alpha, beta, gamma, dh);

    fprintf('Computed Joint Angles for Points A, B, C:\n');
    disp('Point A Joint Angles:');
    disp(thAs);
    disp('Point B Joint Angles:');
    disp(thBs);
    disp('Point C Joint Angles:');
    disp(thCs);

    
    thAs = modulate_angles(thAs, dh);
    thBs = modulate_angles(thBs, dh);
    thCs = modulate_angles(thCs, dh);

    combination = select_combination(thAs, thBs, thCs);
    disp('Selected Combination of Joint Angles:');
    disp(combination);

    times = 0:pat.ts:(pat.tdab + pat.tdbc);
    thetas = zeros( length(times), size(combination, 2) );

    for idx = 1:length(times)
        thetas(idx, : ) = compute_theta(times(idx), pat.tacc, pat.tdab, pat.tdbc, combination(1,:), combination(2,:), combination(3,:), pat.ts);

    end 
    
    disp('Computed Joint Angles over Time:');
    disp(thetas);

    figure;
    plot(times, thetas(:,1), 'r', ...
         times, thetas(:,2), 'g', ...
         times, thetas(:,3), 'b', ...
         times, thetas(:,4), 'c', ...
         times, thetas(:,5), 'm', ...
         times, thetas(:,6), 'y');
    title('Joint Angles over Time');
    xlabel('Time (s)');
    ylabel('Joint Angles (degrees)');
    legend('Theta 1', 'Theta 2', 'Theta 3', 'Theta 4', 'Theta 5', 'Theta 6');

    cartesians = zeros(length(times), 6);
    for idx = 1:length(times)
        [x,y,z, alpha, beta, gamma] = forward(thetas(idx,1), thetas(idx,2), thetas(idx,3), ...
            thetas(idx,4), thetas(idx,5), thetas(idx,6), dh);
            cartesians(idx, :) = [x, y, z, alpha, beta, gamma];
    end
    figure;
    plot(times, cartesians(:,1), 'r', ...
         times, cartesians(:,2), 'g', ...
         times, cartesians(:,3), 'b');
    title('End-Effector Positionover Time');
    xlabel('Time (s)');
    ylabel('Position (m) / Orientation (degrees)');
    legend('X Position', 'Y Position', 'Z Position'); 

    figure;
    plot(times, cartesians(:,4), 'c', ...
         times, cartesians(:,5), 'm', ...
         times, cartesians(:,6), 'y');
    title('End-Effector Orientation over Time');
    xlabel('Time (s)');
    ylabel('Orientation (degrees)');
    legend('Alpha', 'Beta', 'Gamma');

end
