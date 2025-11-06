function right = areAnglesRight(thetas, dh_params)
% Check if the given joint angles are within the specified limits.
% Input: Joint angles (thetas) in degrees and dh_params (struct)
% Output: Boolean right (true if all angles are within limits, false otherwise)

    right = true;
    for i = 1:length(thetas)
        if thetas(i) < rad2deg(dh_params.th_limits(i, 1)) || thetas(i) > rad2deg(dh_params.th_limits(i, 2))
            right = false;
            return;
        end
    end
end