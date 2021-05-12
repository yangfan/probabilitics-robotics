# FastSLAM

The FastSLAM algorithm implemented in this project solves the full SLAM problem. It uses particle filters for estimating the robot path while each particle (which is a hypothesis of robot pose) uses separate EKFs to estimate each map feature location, based on the conditional independence between any two disjoint sets of features in the map, given the robot pose. 

The advantage of FastSLAM is 

1. The data association decision can be made on a per-particle basis. The FastSLAM maintains multiple data associations, in contrast to EKF that tracks only one at any time step

2. The particle filter used in FastSLAM allows the algorithm to deal with the highly nonlinear motion models.

![GitHub Logo](demo/fastslam.gif)

The particles with the highest weight are in red color. The arrow refers to the current pose of the particle with the highest weight. All other particles are in gray color (some of paths may be disappear because of the resampling step). The blue crosses represent the ground-true position of the landmarks. The blue dots represent the estimated landmark positions of each particle. The ellipse is the error ellipse of estimated landmark positions of the particle with the highest weight.

## FastSLAM Overview

The key steps of FastSLAM are following:

1. Prediction: Extend the path posterior by sampling a new pose for each particle, based on the motion model (odometry motion model in this project).

2. Measurement update: Compute measurement predication by the range-bearing sensor model and update belief of each observed landmark by EKF.

3. Compute the particle weights. 

## Code Explanation

1. `sample_motion_model(odometry, particles)`: Sample a new pose for each particle by the odometry motion model.

2. `measurement_model(particle, landmark)`: Compute the expected measurement for the given landmark and the Jacobian with respect to the landmark. 

3. `eval_sensor_model(sensor_data, particles)`: Update the estimate of the landmark position based on the range-bearing measurement<sup>[1]</sup> and compute the particle weights. 

### References

1. **Introduction to Mobile Robotics**  
   University of Freiburg, Spring, 2020, [http://ais.informatik.uni-freiburg.de/teaching/ss20/robotics/index_en.php](http://ais.informatik.uni-freiburg.de/teaching/ss20/robotics/index_en.php)

2. **Probabilistic Robotics**  
   S. Thrun, W. Burgard, and D. Fox. MIT Press, Cambridge, Mass., (2005)
