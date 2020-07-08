# FCND-3D-Estimation
The goal of this project is develop the estimation part of the controller with the help of a the CPP simulator. A succesful implementation would see the controller and the estimation blocks work together to make the drone fly as intended. Essentially we are going to use various sensor measurements from IMU units to have a better estimation of the position, velocity and yaw angle.
## Simulator and C++ Implementation
Most of the code in this project can be found in [FCND-Estimation-CPP](https://github.com/bwassim/FCND-3D-Estimation/tree/master/FCND-Estimation-CPP/src) folder. We have already worked on the controller part in previous project and the implementation can be found here [src/QuadControl.cpp](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/tree/master/FCND-Controls-CPP/src). All the configuration files for the controller and the vehicle are in the `config` directory. 
Some function in [QuadEstimatorEKF.cpp](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/src/QuadEstimatorEKF.cpp) needs to be complemented.

The tunning parameters for the controller are given for this project in  [QuadControlParams.txt](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/config/QuadControlParams.txt), however note that we still need to complete this project with our previously tunned parameters in the control project to succesfully meet the requirements.
The tunning parameters for the estimation project can found in [QuadEstimatorEKF.txt](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/config/QuadEstimatorEKF.txt)
### Prerequisites
If you are using a mac, Xcode would be enough to run the project. Otherwise follow the prerequisites in the seed project's [README](https://github.com/udacity/FCND-Estimation-CPP)

