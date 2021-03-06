# Extended Kalman Filter

## Overview

In this project real-time implementation of extented kaman filter is simulated on the simulation platform that is given by Udacity. Pipeline is written in C++. 

The main purpose of this project is to estimate position and velocity of a bicycle moving around a ego vehicle, and the virtual sensors are Lidar and Radar. The sensors are assumed to be mounted at the center of a vehicle.

Lidar measurements are red circles, ladar measurements are blue circles with an arrow pointing in the direction of the observed angle, and estimation markers are green triangles.

![config](./figs/config.png)



Note that blue car indicates our target object whose position and velocity are to be estimated and the Sensor coordinate `{X, Y}` is attatched to our ego vehicle.



## Problem Formulation

### State vector 

States to be estimated are the position and velocity of a target object in both x and y direction. Namely,

![state_vector](https://latex.codecogs.com/gif.latex?x&space;=&space;[p_x\quad&space;p_y\quad&space;v_x\quad&space;v_y]^T)

### State prediction

The position can be represented by integral of velocity. It can be written in a discrete time as `p(k+1) = p(k)+dt*v(k)`. Implementing this relationship, the prediction model can be derived as follow:

<img src="./figs/state_prediction.png" alt="state_predcition" style="zoom:40%;" />

The above equation is `xdot =Fx + noise`. The model assumes veloicty is constant between time intervals, but in reality we know that an object's velocity can change due to acceleration. The model includes this uncertainty via the process noise. Derivation of process noise and process covariance matrix `Q` are as follow:

![process_noise](./figs/process_noise.png)

### Prediction stage

---

![prediction](https://latex.codecogs.com/gif.latex?x'&space;=&space;Fx\\;P'&space;=&space;FPF^T&space;&plus;&space;Q)

---

## Measurement Update

When updating measurements of sensors, the models of lidar and radar are different. Specifically, lidar model is linear, but the radar model imposes nonlinearity. This is why we use EKF rather than use classical KF.

### Lidar

Lidar sensor directely measures the point cloud of objects respect to the ego vehicle. This gives us a linear relationship, which is simply represented as follow:

![Lidar](https://latex.codecogs.com/gif.latex?z=[p_x\quad&space;p_y]^T=H_{lidar}x\\&space;H_{lidar}=\begin{bmatrix}1&0&0&0\\&space;0&1&0&0\end{bmatrix})



### Radar

Radar sensor measures coordinates of range, bearing and range rate. This measurement has nonliear relationship with the state vector. This relationship can be represented as follow:

![radar_h](https://latex.codecogs.com/gif.latex?h(x)=&space;\begin{bmatrix}&space;\rho\\&space;\phi\\&space;\dot{\rho}&space;\end{bmatrix}=&space;\begin{bmatrix}&space;\sqrt{p_x^2&plus;p_y^2}\\&space;\text{atan2}(p_y,p_x)\\&space;{p_xv_x&plus;p_xv_y}\over{\sqrt{p_x^2&plus;p_y^2}}&space;\end{bmatrix})

Hence for radar `y = z - Hx` becomes `y = z - h(x)`. Extented Kalman filter is a just extended version of Kalman filter for handling this kind of nonlinear relationship by calculating Jacobian matrix at every estimation step.



* Jacobian matrix of `h(x)`

  After calculating all the partial derivatives, our resulted Jacobian, `Hj` is:

  <img src="./figs/jacobian.png" alt="jacobian" style="zoom:30%;" />



### Correction stage

---

![correction](./figs/correction.png)

---



## Pipeline

### How the Files Relate to Each Other

Here is a brief overview of what happens when you run the code files:

1. `Main.cpp` reads in the data and sends a sensor measurement to `FusionEKF.cpp`
2. `FusionEKF.cpp` takes the sensor data and initializes variables and updates variables. The Kalman filter equations are not in this file. `FusionEKF.cpp` has a variable called `ekf_`, which is an instance of a `KalmanFilter` class. The `ekf_` will hold the matrix and vector values. You will also use the `ekf_` instance to call the predict and update equations.
3. The `KalmanFilter` class is defined in `kalman_filter.cpp` and `kalman_filter.h`. You will only need to modify 'kalman_filter.cpp', which contains functions for the prediction and update steps.



### FusionEKF.cpp

pseudo-code:

```
get Measurement{
	if NOT initialized{
		set initial state}
		
    set F, Q matrix with dt
    DO Prediction
    
    if Measureent is RADAR{
    	set corresponding H, R matrix
    	DO Update EKF
    }
    if Measureent is LIDAR{
    	set corresponding H, R matrix
    	DO Update
    }
}
```



### kalman_filter.cpp

This file contains  functions for prediction and measurement update. Detail of algorithm is followed by previous section of this description.

The only tricky thing that needs to be considered is that  `atan2()` function gives us the range of from -pi to +pi, but the range of bearing angle measurement of RADAR is -2pi to 2pi. My solution of dealing with this is as follow:

* Convert the range of bearing angle measurement using `atan2()`  

* Add or subtract 2*pi to the measurement estimation term near the angle of pi where the sign is changed.

