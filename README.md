# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


*Evgeny Nuger*

---

## Overview

This project implements model-predictive-control for the Udacity autonomous car simulator.

The control models the vehicle trajectory by optimizing the fit of a third order polynomial to the expected vehicle location such that the cross track error (among others) is minimized.

The MPC cost coeffecients of the optimization problem were manually tuned. It was noted that an appropriate speed, number of time steps, and time interval must be set to overcome oscillations in the optimization.