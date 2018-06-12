# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


*Evgeny Nuger*

---

## Overview

This project implements model-predictive-control for the Udacity autonomous car simulator.

#### State
The simulator uses the following model state space:
```cpp
// vehicle position x:
px
// vehicle position y:
py
// vehicle orientation:
psi
// vehicle speed:
v
// cross track error:
cte
// vehicle orientation error:
epsi
```
#### Actuators
The model uses two actuators:
```cpp
// Steering angle:
d
// Throttle:
a
```

#### MPC Overview
The MPC algorithm receives the global waypoints, `ptsx`,`ptsy`, the global vehicle position, `px`, `py`, vehicle speed, `v`, throttle, `a`, orientation, `psi`, and steering angle. 
The global waypoints are converted into the vehicle's reference frame (that is aligned with the vehicle) through a translation and rotation operation:
```cpp
p_car = R*[p_waypoint - p_vehicle]
```
where R is the rotation matrix based on the global vehicle orientation angle `psi`, `p_waypoint` is the global waypoint position, `p_vehicle` is the global vehicle position, and `p_car` is the global waypoints in the car's reference frame.

A third-order ploynomial is fit to the waypoints in the vehicles reference frame, and the cross track error is valuated at the origin.

The orientation error is calculated at the origin as well.

To compensate for actuator latency is accounted for through projection of each state with a change in time:
```cpp
// projected x-position from 0:
double px_p = v * cos(psi) * dt;
// project y-position from 0:
double py_p = v * sin(psi) * dt;
// projected orientation:
double psi_p = - v * steering_angle / Lf * dt;
// projected velocity:
double v_p = v + a*dt;
// projected cross track error:
double cte_p = cte + v * sin(epsi) * dt;
// projected orientation error:
double epsi_p = epsi - v/Lf * steering_angle * dt;

``` 

The latency value, `dt`, was set to 150ms to compensate for latency on slower devices.

The projected states are then used with the MPC solver. This solver uses 10 time-steps with a change in time of 0.1s between each step. This was a common, conservative, and low-processor intensive choice.
The use of more timesteps may not be beneficial since the MPC solver calculates a new trajectory at every time step.

The MPC cost coeffecients of the optimization problem were manually tuned. It was noted that an appropriate speed, number of time steps, and time interval must be set to overcome oscillations in the optimization.