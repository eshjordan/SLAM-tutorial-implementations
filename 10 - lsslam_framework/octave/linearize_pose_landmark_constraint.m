% Compute the error of a pose-landmark constraint
% x 3x1 vector (x,y,theta) of the robot pose
% l 2x1 vector (x,y) of the landmark
% z 2x1 vector (x,y) of the measurement, the position of the landmark in
%   the coordinate frame of the robot given by the vector x
%
% Output
% e 2x1 error of the constraint
% A 2x3 Jacobian wrt x
% B 2x2 Jacobian wrt l
function [e, A, B] = linearize_pose_landmark_constraint(x, l, z)

  % TODO compute the error and the Jacobians of the error
  % My Note: See the Graph-based SLAM tutorial paper
   
  Ri = [
    cos(x(3)), -sin(x(3));
    sin(x(3)), cos(x(3))
  ];
  dRi = [
    -sin(x(3)), -cos(x(3));
    cos(x(3)), -sin(x(3))
  ];
  e = Ri'*(l - x(1:2)) - z;
  A = [-Ri', dRi'*(l - x(1:2))];
  B = Ri';

end;
