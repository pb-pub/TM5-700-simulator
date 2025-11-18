function best_combination = select_combination(thetas)

%  best_combination = SELECT_COMBINATION(thetas) selects the best combination
%  of joint angles from a list of possible solutions based on the cost function.
%  This is a simple minimization approach that selects the combination with the lowest cost
%  at each point in space, considering the previous selection to ensure smooth transitions.
%  Input:
%      thetas - 3D matrix where each row represents a set of joint angles (in degrees)
%               Each slice along the third dimension corresponds to a different point in space.
%  Output:
%      best_combination - Matrix of joint angles (in degrees) representing the best combination.
%                      Each row corresponds to a point in space.

num_points = size(thetas, 3);
best_combination = zeros(num_points, size(thetas, 2));
for i = 1:num_points
    min_cost = inf;
    best_thetas = [];
    for j = 1:size(thetas, 1)
        current_thetas = squeeze(thetas(j, :, i));
        if all(current_thetas ~= inf)
            if i == 1
                cost_value = cost(current_thetas, zeros(1, size(thetas, 2)));
            else
                cost_value = cost(current_thetas, best_combination(i-1, :));
            end
            
            if cost_value < min_cost
                min_cost = cost_value;
                best_thetas = current_thetas;
            end
        end
    end
    best_combination(i, :) = best_thetas;
end

end