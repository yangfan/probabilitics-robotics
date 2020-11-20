function [mu, sigma, observedLandmarks] = correction_step(mu, sigma, z, observedLandmarks)
% Updates the belief, i. e., mu and sigma after observing landmarks, according to the sensor model
% The employed sensor model measures the range and bearing of a landmark
% mu: 2N+3 x 1 vector representing the state mean.
% The first 3 components of mu correspond to the current estimate of the robot pose [x; y; theta]
% The current pose estimate of the landmark with id = j is: [mu(2*j+2); mu(2*j+3)]
% sigma: 2N+3 x 2N+3 is the covariance matrix
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector observedLandmarks indicates which landmarks have been observed
% at some point by the robot.
% observedLandmarks(j) is false if the landmark with id = j has never been observed before.

% Number of measurements in this time step
m = size(z, 2);

% Z: vectorized form of all measurements made in this time step: [range_1; bearing_1; range_2; bearing_2; ...; range_m; bearing_m]
% ExpectedZ: vectorized form of all expected measurements in the same form.
% They are initialized here and should be filled out in the for loop below
Z = zeros(m*2, 1);
expectedZ = zeros(m*2, 1);

% Iterate over the measurements and compute the H matrix
% (stacked Jacobian blocks of the measurement function)
% H will be 2m x 2N+3
H = [];

x = mu(1)
y = mu(2)
theta = mu(3)

for i = 1:m
	% Get the id of the landmark corresponding to the i-th observation
	landmarkId = z(i).id;
  r = z(i).range;
  phi = z(i).bearing;
	% If the landmark is obeserved for the first time:
	if(observedLandmarks(landmarkId)==false)
		% Initialize its pose in mu based on the measurement and the current robot pose:
    mu(2*landmarkId + 2) = x + r * cos(theta + phi);
    mu(2*landmarkId + 3) = y + r * sin(theta + phi);
		% Indicate in the observedLandmarks vector that this landmark has been observed
		observedLandmarks(landmarkId) = true;
	endif

	% Add the landmark measurement to the Z vector
	Z(2*i - 1 : 2*i) = [r; phi];
	% Use the current estimate of the landmark pose
	% to compute the corresponding expected measurement in expectedZ:
  lx = mu(2*landmarkId + 2);
  ly = mu(2*landmarkId + 3);
  
  r_expected = sqrt((lx - x)^2 + (ly - y)^2);
  phi_expected = normalize_angle(atan2(ly - y, lx - x) - theta);
  
  expectedZ(2*i - 1 : 2*i) = [r_expected; phi_expected];
	% Compute the Jacobian Hi of the measurement function h for this observation
	Hi = zeros(2, size(mu, 1));
  Hi(:, 1:3) = [(x - lx) / r_expected, (y - ly) / r_expected, 0;
                  (ly - y) / (r_expected^2), (x - lx) / (r_expected^2), -1];
  idx = 2*landmarkId + 2;
  Hi(:, idx : idx + 1) = [(lx - x) / r_expected, (ly - y) / r_expected;
    (y - ly) / (r_expected^2), (lx - x) / (r_expected^2)];
   
	% Augment H with the new Hi
	H = [H;Hi];	
endfor

% Construct the sensor noise matrix Q
Q = 0.01 * eye(2*m);
% Compute the Kalman gain
K = (sigma * H') * inv(H * sigma * H' + Q);
% Compute the difference between the expected and recorded measurements.
% Remember to normalize the bearings after subtracting!
delta = normalize_all_bearings(Z - expectedZ);
% TODO: Finish the correction step by computing the new mu and sigma.
% Normalize theta in the robot pose.
mu_corrected = mu + K * delta;
sigma_corrected = (eye(size(mu, 1)) - K * H) * sigma;

mu = mu_corrected;
sigma = sigma_corrected;

end
