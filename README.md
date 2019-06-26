# Extended Kalman Filter Project
Self-Driving Car Engineer Nanodegree Program

[//]: # (Image References)

[image1]: ./images/data1.png "Dataset 1"
[image2]: ./images/data2.png "Dataset 2"

This project contains my solution to Project 5 of Udacity's Self Driving Car Nanodegree. The original source code as well as build instructions can be found [in the original repository](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project). 

In this project we utilize a Kalman filter to estimate the state of a moving object of interest with noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower than the tolerance outlined in the project rubric. 

The Kalman filter is tested with Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases).

Videos of the 2 datasets can be found here:

[![Dataset 1][image1]](https://www.youtube.com/watch?v=PVazTHXptWA)
[![Dataset 2][image2]](https://www.youtube.com/watch?v=NSlQj-JGBfg)

The final Root Mean Square Errors are:

| value| Dataset 1 | Dataset 2 |
|-----|-----------:|-----------:|
| x   | 0.0965    | 0.0743    |
| y   | 0.0854    | 0.0946    |
| v_x | 0.4022    | 0.4072    |
| v_y | 0.4606    | 0.4522    |