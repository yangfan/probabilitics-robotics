The Kalman filter is a technique for filtering and prediction in linear Gaussian systems. It is assumed that

1. The initial belief is normally distributed.

2. The motion model is a linear function with additional Gaussian noise.

3. The observation model is linear with Gaussian noise.

These assumptions are crucial for the correctness of the KF. The efficiency of KF is due to the fact that any linear transformation of a Gaussian random variable results in another Gussian random variable which can be computed in closed form.

However, in most robotics system, the motion model and observation model are nonlinear. To deal with the nonlinearity, the Extended Kalman Filter linearizes the nonlinear function by the first order Taylor expansion and computes a Gussian approximation to the true belief. 

In this project, I implemented the EKF in Python. Given the ground-true position of the landmark, sensor readings and odometry data, at each time step the robot estimate the current pose parameterized by mean and covariance of a Gaussian distribution. The belief is visualized by the arrow indicating the mean pose and covariance ellipse.

![GitHub Logo](demo/kalman_filter.gif)

## EKF Overview

The extended Kalman Filter relaxes one of the assumptions, the linearity assumption. More specifically, the linear system matrices in motion model and observation model are now replaced by nonlinear functions. The nonlinear functions are approximated by linear functions using first-order Taylor explansion.

The EKF basically replace the linear prediction by the nonlinear generalizations in EKF. In particular, the linear system matrices are replaced by the Jacobian of the nonlinear functions.

## Code Explanation

1. `prediction_step(odometry, mu, sigma)`: given the odometry and the estimate of the previous pose, motion noise, return the predicted pose in the current time step using the Jacobian of the noise-free odometry motion model. 


2. `correction_step(sensor_data, mu, sigma, landmarks)`: given the sensor readings, the ground-truth position of the landmarks and the predicated pose of current time step, return the corrected pose using Jacobian.

### References

1. **Introduction to Mobile Robotics**  
   University of Freiburg, Spring, 2020, [http://ais.informatik.uni-freiburg.de/teaching/ss20/robotics/index_en.php](http://ais.informatik.uni-freiburg.de/teaching/ss20/robotics/index_en.php).

2. **Probabilistic Robotics**  
   S. Thrun, W. Burgard, and D. Fox. MIT Press, Cambridge, Mass., (2005)