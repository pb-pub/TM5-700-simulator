function best_combination = select_combination(thAs, thBs, thCs)
%  best_combination = SELECT_COMBINATION(thetas) selects the best combination
%  of joint angles from a list of possible solutions based on the cost function.
%  This is a simple minimization approach that selects the combination with the lowest cost
%  at each point in space, considering the previous selection to ensure smooth transitions.
%  It returns a 2D matrix where each row corresponds to a point in space. The cost of a 
% combination is the sum of the weighted squared differences between consecutive joint angle sets.
% Input:
%   thAs, thBs, thCs - (N x 6) matrices containing possible joint solutions 
%                      for points A, B, and C respectively.
% Output:
%   best_combination - (3 x 6) matrix with the selected configuration for A, B, C.

    % Get the number of solutions for each point
    num_sol_A = size(thAs, 1);
    num_sol_B = size(thBs, 1);
    num_sol_C = size(thCs, 1);

    % Initialize minimum cost to infinity
    min_total_cost = inf;
    
    % Store best indices: [idx_A, idx_B, idx_C]
    best_indices = [1, 1, 1];

    % --- EXHAUSTIVE SEARCH ---
    for i = 1:num_sol_A
        sol_A = thAs(i, :);
        if any(isnan(sol_A)), continue; end % Skip invalid solutions

        for j = 1:num_sol_B
            sol_B = thBs(j, :);
            if any(isnan(sol_B)), continue; end
            
            % Calculate cost A -> B
            cost_AB = cost(sol_A, sol_B);

            for k = 1:num_sol_C
                sol_C = thCs(k, :);
                if any(isnan(sol_C)), continue; end

                % Calculate cost B -> C
                cost_BC = cost(sol_B, sol_C);

                % Total Cost
                current_total_cost = cost_AB + cost_BC;

                % Check if this is the new minimum
                if current_total_cost < min_total_cost
                    min_total_cost = current_total_cost;
                    best_indices = [i, j, k];
                end
            end
        end
    end

    % --- CONSTRUCT OUTPUT ---
    % Create the 3x6 matrix with the best solutions found
    best_combination = zeros(3, 6);
    best_combination(1, :) = thAs(best_indices(1), :);
    best_combination(2, :) = thBs(best_indices(2), :);
    best_combination(3, :) = thCs(best_indices(3), :);

end
