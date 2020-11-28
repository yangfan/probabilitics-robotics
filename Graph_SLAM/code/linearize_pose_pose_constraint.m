% Compute the error of a pose-pose constraint
% x1 3x1 vector (x_1,y_1,theta_1) of the first robot pose
% x2 3x1 vector (x_2,y_2,theta_2) of the second robot pose
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

  % compute the error and the Jacobians of the error
  x_1 = x1(1);
  y_1 = x1(2);
  theta_1 = x1(3);
  x_2 = x2(1);
  y_2 = x2(2);
  theta_2 = x2(3);
  
  Q_1 = [-cos(theta_1), -sin(theta_1), (x_1 - x_2)*sin(theta_1) + (y_2 - y_1)*cos(theta_1);
          sin(theta_1), -cos(theta_1), (x_1 - x_2)*cos(theta_1) + (y_1 - y_2)*sin(theta_1)];
  
  Q_2 = [cos(theta_1), sin(theta_1), 0;
        -sin(theta_1), cos(theta_1), 0];
        
  Z = v2t(z);
  X1 = v2t(x1);
  X2 = v2t(x2);
  
  A = [Z(1:2,1:2)' * Q_1; 0, 0, -1];
  B = [Z(1:2,1:2)' * Q_2; 0, 0, 1];
  
  e = [Z(1:2,1:2)' * (X1(1:2,1:2)' * (x2(1:2) - x1(1:2)) - z(1:2));
       normalize_angle(x2(3) - x1(3) - z(3))];
  
          

end;
