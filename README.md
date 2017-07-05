# Unscented_Kalman_Filter_Project

Built an Unscented Kalman filter using C++ to estimate the state of a moving object of interest with noisy lidar and radar measurements

A standard Kalman filter can only handle linear equations. Both the extended Kalman filter and the unscented Kalman filter allow you to use non-linear equations; the difference between EKF and UKF is how they handle non-linear equations. But the basics are the same: initialize, predict, update.

All Kalman filters have the same three steps:

1. Initialization
2. Prediction
3. Update

***

### Files in the GitHub src Folder

**main.cpp** - reads in data, calls a function to run the Kalman filter, calls a function to calculate RMSE

**ukf.cpp** - initializes the filter, calls the predict and update function, defines the predict and update functions

**tools.cpp** - function to calculate RMSE

***

### Data

The data file information is provided by the simulator and is the same data files from EKF. Again each line in the data file represents either a lidar or radar measurement marked by "L" or "R" on the starting line. The next columns are either the two lidar position measurements (x,y) or the three radar position measurements (rho, phi, rho_dot). Then comes the time stamp and finally the ground truth values for x, y, vx, vy, yaw, yawrate.

Although the data set contains values for yaw and yawrate ground truth, there is no need to use these values. main.cpp does not use these values, and you are only expected to calculate RMSE for x, y vx and vy. You can compare your vx and vy RMSE values from the UKF project and the EKF project. For UKF, vx and vy RMSE should be lower than for EKF; this is because we are using a more detailed motion model and UKF is also known for handling non-linear equations better than EKF.
