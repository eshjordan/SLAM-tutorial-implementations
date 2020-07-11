function x = motion_function(x,u)
%MOTION_FUNCTION Motion Function
x = x + [
    u(2)*cos(x(3) + u(1));
    u(2)*sin(x(3) + u(1));
    u(1) + u(3)
];
end
