# Unscented Kalman Filter

Generally speaking, the Kalman filters require linear motion model and sensor model to perform the transformation of Gaussian distribution. In EKF, the first order Taylor expansion is used to linearize the functions at the mean of the belief. An alternative linearization method, called unscented transform is used in the Unscented Kalman Filter. In particular, the UKF extracts a set of points with weights from the Gaussian and passes those points through the nonlinear function. The transformed Gaussian can be recovered from those returned points. 

In this project, we implemented a UKF for the online SLAM problem. Similar to the EKF, given the sensor reading, odometry information and the data association<sup>[1]</sup>, the robot estimates the its pose (shown by the red covariance ellipse and bar), the position of map feastures (shown by the blue covariance ellipse) at each time step.

![GitHub Logo](demo/ukf_slam.gif)

## UKF Overview

The points extracted from a Gaussian is called the sigma points, denoted by $$\chi^{[i]}$$ and weight $$w_m^{[i]}$$ and $$w_c^{[i]}$$. The transformed Gaussian distribution can then be recovered from the returned sigma points $$\mathcal{Y}^{[i]}$$.

Using the transformation mentioned above, the predication step in EKF can be replaced by the sigma point propagation of the motion model. Similarily in correction step we can pass the obtained sigma points (of the predicted Gaussian) to the sensor model $$h(x)$$, and recover the mean and covariance of the expected observation. The Kalman gain then can be computed from the covariance of the observation and the cross-covariance between state and observation. 

## Code Explanation

1. `[mu, sigma, sigma_points] = prediction_step(mu, sigma, u)`: impelment the predication step, compute the mean and covariance of the state which includes the robot's pose and landmark positions. 

2. `[mu, sigma, map] = correction_step(mu, sigma, z, map)`: implement the correction step, update the mean and covariance of state after the range-bearing sensor measurement.

### References

1. **Robot Mapping**  
   University of Freiburg, WS 2013/14, [http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/](http://ais.informatik.uni-freiburg.de/teaching/ws13/mapping/)

2. **Probabilistic Robotics**  
   S. Thrun, W. Burgard, and D. Fox. MIT Press, Cambridge, Mass., (2005)
