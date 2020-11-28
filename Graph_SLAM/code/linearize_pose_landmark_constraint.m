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

  % compute the error and the Jacobians of the error
  X = v2t(x);
  
  e = X(1:2,1:2)' * (l - x(1:2)) - z;
  
  x_i = x(1);
  y_i = x(2);
  theta = x(3);
  l_x = l(1);
  l_y = l(2);
  A = [-cos(theta), -sin(theta), (x_i - l_x) * sin(theta) + (l_y - y_i) * cos(theta);
        sin(theta), -cos(theta), (x_i - l_x) * cos(theta) + (y_i - l_y) * sin(theta)];
  B = [cos(theta), sin(theta); -sin(theta), cos(theta)];      

end;
