function [T] = mdh(a,alpha,d,theta)
% Modified Denavit-Hartenberg transformation matrix
% Input: a, alpha, d (in radians), theta (in radians)
% Output: 4x4 transformation matrix T
    T = [cos(theta), -sin(theta), 0, a;
         sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d;
         sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d;
         0, 0, 0, 1];

end