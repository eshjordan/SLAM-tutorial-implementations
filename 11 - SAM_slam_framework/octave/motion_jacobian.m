function [F, G] = motion_jacobian(x,u)
%MOTION_JACOBIAN Summary of this function goes here
%   Detailed explanation goes here
F = [
    1, 0, -u(2)*sin(x(3) + u(1));
    0, 1,  u(2)*cos(x(3) + u(1));
    0, 0, 1;
];
G = -eye(3);
end

