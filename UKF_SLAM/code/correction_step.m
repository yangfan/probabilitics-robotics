function [mu, sigma, map] = correction_step(mu, sigma, z, map);
% Updates the belief, i.e., mu and sigma after observing landmarks,
% and augments the map with newly observed landmarks.
% The employed sensor model measures the range and bearing of a landmark
% mu: state vector containing robot pose and poses of landmarks obeserved so far.
% Current robot pose = mu(1:3)
% Note that the landmark poses in mu are stacked in the order by which they were observed
% sigma: the covariance matrix of the system.
% z: struct array containing the landmark observations.
% Each observation z(i) has an id z(i).id, a range z(i).range, and a bearing z(i).bearing
% The vector 'map' contains the ids of all landmarks observed so far by the robot in the order
% by which they were observed, NOT in ascending id order.

% For computing sigma
global scale;

% Number of measurements in this time step
m = size(z, 2);

% Measurement noise
Q = 0.01*eye(2);

for i = 1:m

	% If the landmark is observed for the first time:
	if (isempty(find(map == z(i).id)))
	    % Add new landmark to the map
      [mu, sigma, map] = add_landmark_to_map(mu, sigma, z(i), map, Q);
	    % The measurement has been incorporated so we quit the correction step
	    continue;
	endif

	% Compute sigma points from the predicted mean and covariance
	sigma_points = compute_sigma_points(mu, sigma);
	sigma_points(3,:) = normalize_angle(sigma_points(3,:));

	% Compute lambda
	n = length(mu);
	num_sig = size(sigma_points,2);
	lambda = scale - n;

  % extract the current location of the landmark for each sigma point
  % Use this for computing an expected measurement, i.e., applying the h function
	landmarkIndex = find(map==(z(i).id));
	landmarkXs = sigma_points(2*landmarkIndex + 2, :);
	landmarkYs = sigma_points(2*landmarkIndex + 3, :);

	% Compute z_points (2x2n+1), which consists of predicted measurements from all sigma points
  r_pts = sqrt((landmarkXs - sigma_points(1,:)).^2 + (landmarkYs - sigma_points(2,:)).^2);
  phi_pts = normalize_angle(atan2(landmarkYs - sigma_points(2,:), landmarkXs - sigma_points(1,:)) - sigma_points(3,:));
  z_points = [r_pts; phi_pts];
  % setup the weight vector for mean and covariance
  wm = [lambda/scale, repmat(1/(2*scale), 1, 2*n)];
  wc = wm;

	% Compute zm
	% zm is the recovered expected measurement mean from z_points.
	% It will be a 2x1 vector [expected_range; expected_bearing].
        % For computing the expected_bearing compute a weighted average by
        % summing the sines/cosines of the angle
   zm = z_points * wm';
   sum_cos = cos(z_points(2,:)) * wm';
   sum_sin = sin(z_points(2,:)) * wm';
   zm(2) = normalize_angle(atan2(sum_sin, sum_cos));


	% Compute the innovation covariance matrix S (2x2), line 9 on slide 32
        % Remember to normalize the bearing after computing the difference
  diff_zz = z_points - zm;
  diff_zz(2,:) = normalize_angle(diff_zz(2,:));
  S = wc .* diff_zz * diff_zz' + Q;

	% Compute Sigma_x_z, line 10 on slide 32
        % (which is equivalent to sigma times the Jacobian H transposed in EKF).
	% sigma_x_z is an nx2 matrix, where n is the current dimensionality of mu
        % Remember to normalize the bearing after computing the difference
  diff_mumu = sigma_points - mu;
  diff_mumu(3,:) = normalize_angle(diff_mumu(3,:));
  Sigma_x_z = wc .* diff_mumu * diff_zz';

	% Compute the Kalman gain
  K = Sigma_x_z * inv(S);

	% Get the actual measurement as a vector (for computing the difference to the observation)
	z_actual = [z(i).range; z(i).bearing];

	% Update mu and sigma
        % normalize the relative bearing
  delta_z = z_actual - zm;
  delta_z(2) = normalize_angle(delta_z(2));
  mu += K * delta_z;
  sigma += -K * S * K';
	% TODO: Normalize the robot heading mu(3)
  mu(3) = normalize_angle(mu(3));

endfor

end
