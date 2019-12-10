# Kalman Filter + Kimera
---
This repository implements a Kalman Filter for prefiltering IMU measurements from a tracked rover robot. The robot model was implemented in Matlab and autocoded to C++ for inclusion here. The filtered IMU can then be given to a VIO pipline, such as [Kimera](https://github.com/MIT-SPARK/Kimera-VIO-ROS), in order to incorporate vehicle dynamics into pose estimates. 

## Configuration and Dependencies
---
This project is meant to be used with catkin and ROS melodic. It depends on having [catkin_eigen](https://github.com/ethz-asl/eigen_catkin) and [catkin_simple](https://github.com/catkin/catkin_simple) in the same catkin workspace as this project.
