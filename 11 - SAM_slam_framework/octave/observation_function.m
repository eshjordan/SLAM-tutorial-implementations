function z = observation_function(x, z, bearing)
%OBSERVATION_FUNCTION
if ~bearing
    z = x(1:2) + z(1)*[cos(z(2) + x(3));sin(z(2) + x(3))];
end
