function [H, J] = observation_jacobian(x, z, bearing)
%OBSERVATION_JACOBIAN Summary of this function goes here
%   Detailed explanation goes here
if bearing
    J = [
        cos(z(2)), sin(z(2));
        -(1/z(1))*sin(z(2)), (1/z(1))*cos(z(2))
    ];

else
    d = z - x;
    q = d'*d;
    J = [
        d(1)/sqrt(q), d(2)/sqrt(q);
        -d(2)/q, d(1)/q;
    ];

end

H = [-J, [0;-1]];

end
