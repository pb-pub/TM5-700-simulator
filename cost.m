function c = cost(thetas, thetas2)
%   c = COST(thetas, thetas2) computes a weighted squared difference
%   between two vectors of joint angles thetas and thetas2.
%   The weights decrease linearly from the first to the last joint.
%   Input:
%       thetas  - vector of joint angles (in degrees)
%       thetas2 - vector of joint angles (in degrees)
%   Output:
%       c       - computed cost value

c=0;
for k = 1:length(thetas)
    diff = abs(thetas(k) - thetas2(k));

    c = c + (length(thetas) + 1 - k)*diff^2;
end


end