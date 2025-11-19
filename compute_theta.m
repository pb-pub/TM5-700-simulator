function theta = compute_theta(t, tacc, tdab, tdbc, thA, thB, thC, ts)
% Compute the joint angles at time t given the necessary parameters.
% This is a recursive function using the derivation in the report.
% Input:
%   t     - Current time
%   tacc  - half transition time
%   tdab  - time from A to B
%   tdbc  - time from B to C
%   thA   - Initial joint angle (degrees)
%   thB   - Intermediate joint angle (degrees)
%   thC   - Final joint angle (degrees) 
%   ts    - time step
% Output:
%   theta - Joint angle at time t (degrees)

tab = tdab - 2*tacc; 
tbc = tdbc - 2*tacc;

thpbc = (thC - thB) / (tdbc - 1/2*tacc); % Theta dot from B to C
thpab = (thB - thA) / (tdab - 1/2*tacc); % Theta dot from A to B

thppa = thpab / tacc; % Theta double dot from A to B
thppc = - thpbc / tacc; % Theta double dot from B to C
thppb = (thpbc - thpab) / (2*tacc); % Theta double dot at B

if t<=0
    theta = thA;
elseif t > 0 && t <= tacc
    theta = thA + t^2/2 * thppa;
elseif t > tacc && t <= (tacc + tab)
    start_theta = compute_theta(tacc, tacc, tdab, tdbc, thA, thB, thC, ts);
    theta = start_theta + thpab * (t - tacc);
elseif t > (tacc + tab) && t <= (tacc + tab + tacc)
    start_theta = compute_theta(tacc + tab, tacc, tdab, tdbc, thA, thB, thC, ts);
    delta_t = t - (tacc + tab);
    theta = start_theta + thpab * delta_t + 1/2 * thppb * delta_t^2;
elseif t > (tacc + tab + tacc) && t <= (tacc + tab + tacc + tbc)
    start_theta = compute_theta(tacc + tab + tacc, tacc, tdab, tdbc, thA, thB, thC, ts);
    delta_t = t - (tacc + tab + tacc);
    theta = start_theta + thpbc * delta_t;
elseif t > (tacc + tab + tacc + tbc) && t <= (tdab + tdbc)
    start_theta = compute_theta(tacc + tab + tacc + tbc, tacc, tdab, tdbc, thA, thB, thC, ts);
    delta_t = t - (tacc + tab + tacc + tbc);
    theta = start_theta + thpbc * delta_t + 1/2 * thppc * delta_t^2;
else
    theta = thC;

end 
end
