% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  % My Note: See the Graph-based SLAM tutorial paper
  
  % Extract the needed rotation matrices
##  Xi = v2t(x1);
##  Xj = v2t(x2);
##  Z = v2t(z);
##  Ri = Xi(1:2, 1:2);
##  dRi = [
##    -sin(x1(3)), cos(x1(3));
##    cos(x1(3)), -sin(x1(3))
##  ];
##  Rj = Xj(1:2, 1:2);
##  Rij = Z(1:2, 1:2);
##  ti = Xi(1:2, 3);
##  tj = Xj(1:2, 3);
##  tij = Z(1:2, 3);
##  
##  % Compute the error
##  e = [
##    Rij'*(Ri'*(tj - ti) - tij);
##    x2(3) - x1(3) - z(3)
##  ];
  
  Ri = [
    cos(x1(3)), -sin(x1(3));
    sin(x1(3)), cos(x1(3))
  ];
  dRi = [
    -sin(x1(3)), -cos(x1(3));
    cos(x1(3)), -sin(x1(3))
  ];
  Rij = [
    cos(z(3)), -sin(z(3));
    sin(z(3)), cos(z(3))
  ];
  
##   Compute the error
  e = [
    Rij'*(Ri'*(x2(1:2) - x1(1:2)) - z(1:2));
    x2(3) - x1(3) - z(3)
  ];
  
  % Derivative w.r.t. x1
  A = [
    -Rij'*Ri', Rij'*dRi'*(x2(1:2) - x1(1:2));
    0, 0, -1
  ];
  
  % Derivative w.r.t. x2
  B = [
    Rij'*Ri', [0;0];
    0, 0, 1
  ];

end;
