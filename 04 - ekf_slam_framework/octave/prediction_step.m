function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% TODO: Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update (hint: use the function normalize_angle available in tools)
mu(1:3) = mu(1:3) + [
  u.t * cos(mu(3) + u.r1);
  u.t * sin(mu(3) + u.r1);
  normalize_angle(u.r1 + u.r2)
];

% TODO: Compute the 3x3 Jacobian Gx of the motion model
Gx = eye(3);
Gx(1:2, 3) = [-u.t * sin(mu(3) + u.r1);u.t * cos(mu(3) + u.r1)]';

% TODO: Construct the full Jacobian G
##G = eye(size(sigma));
##G(1:3, 1:3) = Gx;

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
##R = zeros(size(sigma,1));
##R(1:3,1:3) = R3;

% TODO: Compute the predicted sigma after incorporating the motion
sigma(1:3, 1:3) = Gx*sigma(1:3, 1:3)*Gx' + R3;
sigma(1:3, 4:end) = Gx*sigma(1:3, 4:end);
sigma(4:end, 1:3) = sigma(1:3, 4:end)';

end
