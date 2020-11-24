function [mu, sigma, sigma_points] = prediction_step(mu, sigma, u)
% Updates the belief concerning the robot pose according to the motion model.
% mu: state vector containing robot pose and poses of landmarks obeserved so far
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% u: odometry reading (r1, t, r2)
% Use u.r1, u.t, and u.r2 to access the rotation and translation values

% For computing lambda.
global scale;


% Compute sigma points
sigma_points = compute_sigma_points(mu, sigma);
sigma_points(3,:) = normalize_angle(sigma_points(3,:));

% Dimensionality
n = length(mu);
% lambda
lambda = scale - n;

% Transform all sigma points according to the odometry command
% Remember to vectorize your operations and normalize angles
% Tip: the function normalize_angle also works on a vector (row) of angles
delta = zeros(n, 2*n + 1);
delta(1:3, :) = [u.t * cos(sigma_points(3,:) + u.r1); u.t * sin(sigma_points(3,:) + u.r1); repmat(u.r1 + u.r2, 1, 2*n + 1)];
sigma_points_trans = sigma_points + delta;
sigma_points_trans(3,:) = normalize_angle(sigma_points_trans(3,:));

% Computing the weights for recovering the mean
wm = [lambda/scale, repmat(1/(2*scale),1,2*n)];
wc = wm;

% recover mu.
% Be careful when computing the robot's orientation (sum up the sines and
% cosines and recover the 'average' angle via atan2)
mu_predicated = sigma_points_trans * wm';
sum_cos = cos(sigma_points_trans(3,:)) * wm';
sum_sin = sin(sigma_points_trans(3,:)) * wm';
mu_predicated(3) = normalize_angle(atan2(sum_sin, sum_cos));

% Recover sigma. Again, normalize the angular difference
diff = sigma_points_trans - mu_predicated;
diff(3,:) = normalize_angle(diff(3,:));
sigma_predicated  = wc .* diff * diff';

% Motion noise
motionNoise = 0.1;
R3 = [motionNoise, 0, 0; 
     0, motionNoise, 0; 
     0, 0, motionNoise/10];
R = zeros(size(sigma,1));
R(1:3,1:3) = R3;

% Add motion noise to sigma
sigma_predicated += R;

mu = mu_predicated;
sigma = sigma_predicated;

end
