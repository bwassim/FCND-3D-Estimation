# FCND-3D-Estimation
The goal of this project is develop the estimation part of the controller with the help of a the CPP simulator. A succesful implementation would see the controller and the estimation blocks work together to make the drone fly as intended. Essentially we are going to use various sensor measurements from IMU units to have a better estimation of the position, velocity and yaw angle.
## Simulator and C++ Implementation
Most of the code in this project can be found in [FCND-Estimation-CPP](https://github.com/bwassim/FCND-3D-Estimation/tree/master/FCND-Estimation-CPP/src) folder. We have already worked on the controller part in previous project and the implementation can be found here [src/QuadControl.cpp](https://github.com/bwassim/FCND-3D-Quadrotor-Controller/tree/master/FCND-Controls-CPP/src). All the configuration files for the controller and the vehicle are in the `config` directory. 
Some function in [QuadEstimatorEKF.cpp](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/src/QuadEstimatorEKF.cpp) needs to be complemented.

The tunning parameters for the controller are given for this project in  [QuadControlParams.txt](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/config/QuadControlParams.txt), however note that we still need to complete this project with our previously tunned parameters in the control project to succesfully meet the requirements.
The tunning parameters for the estimation project can found in [QuadEstimatorEKF.txt](https://github.com/bwassim/FCND-3D-Estimation/blob/master/FCND-Estimation-CPP/config/QuadEstimatorEKF.txt)
### Prerequisites
If you are using a mac, Xcode would be enough to run the project. Otherwise follow the prerequisites in the seed project's [README](https://github.com/udacity/FCND-Estimation-CPP)

## The tasks
In order to complete the building of the estimator, we need to go through and complete the following steps 

* Step 1: Sensor Noise
* Step 2: Attitude Estimation
* Step 3: Prediction Step
* Step 4: Magnetometer Update
* Step 5: Closed Loop + GPS Update
* Step 6: Adding Your Controller

### Step 1: Sensor Noise 

In this section, we the sensor noise deviation for the GPS x signal and the IMU accelerometer x signal logs and compute the standard deviation of each signal.
The calculations and results are in this [jupyter notebook ](https://github.com/bwassim/FCND-3D-Estimation/tree/master/FCND-Estimation-CPP/jupyter-notebook)

<img src="./images/sensor.gif" width=650/>

`
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 68% of the time 
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 69% of the time`

### Step2: Attitude Estimation 
In this section we need to improve the complementary attitude filter. Currently a linearized version based on a small angle approximation is implemented. The integrated (predicted) value is then updated in a complementary filter style with attitude information from accelerometers.
`We basically need to update the attitude estimation with the real nonlinear model instead of the linearized one`. 
To do this we just need to convert the body rates angles p, q, r we obtain from the gyro to angular velocities in the intertial frame. We have already used this transformation in the previous control project. as a reminder we proceed with the following transformation 

<img src="./images/R.png" width=400 />

The predicted pitch and predicted roll are now updated with respect to the transformed gyro angles. The Roll and Pitch angles are derived from the accelerometer. The general expression to calculate the complementary filter remains the same 

<img src="./images/attitude_update.png" width=380 />

With the previous transformation we obtain the result in the following animation 

<img src="./images/attitude_estimation.gi" width=650 />

### Step3: Prediction Step
Run scenario 08_PredictState. This scenario is configured to use a perfect IMU (only an IMU). Due to the sensitivity of double-integration to attitude errors, we've made the accelerometer update very insignificant (QuadEstimatorEKF.attitudeTau = 100). The plots on this simulation show element of your estimated state and that of the true state. At the moment you should see that your estimated state does not follow the true state.

In this section we want to initially implement the prediction step of our EKF filter. The state prediction step is implemented in [FCND-Estimation-CPP/src/QuadEstimatorEKF::PredictState](https://github.com/bwassim/FCND-3D-Estimation/blob/aab30c1da582a3c4dd4f18ce0e2efd2ca8373f50/FCND-Estimation-CPP/src/QuadEstimatorEKF.cpp#L179-L185). It can observed from the simulation below that the estimated state tracks the actual state with small error. 

<img src="./images/prediction.gif" width=650 />

The previous simulation used a perfect IMU. This time we would like to introduce some realistic IMU by introducing some noise.

<img src="./images/xv_est10.jpg" />