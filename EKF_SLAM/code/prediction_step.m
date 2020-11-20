function [mu, sigma] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model,
% mu: 2N+3 x 1 vector representing the state mean
% sigma: 2N+3 x 2N+3 covariance matrix
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% Compute the new mu based on the noise-free (odometry-based) motion model
% Remember to normalize theta after the update
x = mu(1);
y = mu(2);
theta = mu(3);
delta_mu = zeros(size(mu));


delta_mu(1) = u.t * cos(theta + u.r1);
delta_mu(2) = u.t * sin(theta + u.r1);
delta_mu(3) = u.r1 + u.r2;

mu_predicated = mu + delta_mu;
mu_predicated(3) = normalize_angle(mu_predicated(3));

% Compute the 3x3 Jacobian Gx of the motion model
Gx = [1, 0, -u.t * sin(theta + u.r1);
      0, 1,  u.t * cos(theta + u.r1);
      0, 0,  1];

% Construct the full Jacobian G
G = zeros(size(mu,1));
G(1:3, 1:3) = Gx;
G(4:end, 4:end) = eye(size(mu, 1) - 3);

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% Compute the predicted sigma after incorporating the motion
sigma_predicated = G * sigma * G' + R;

mu = mu_predicated;
sigma = sigma_predicated;

end
