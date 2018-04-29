# Model Predictive Controller
This repository contains code for the final project of term two of the [Udacity Self-Driving Car Nanodegree](
https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013).

For this project, I implemented a model predictive controller that drives a simulated car around a virtual track. The
model predictive controller receives a sequence of waypoints that the car should travel through. It fits a polynomial
to the waypoints and uses this polynomial path as its target trajectory. It then solves an optimization problem to find
the series of actuations that will steer the car closest to the target trajectory. To predict what trajectory a set of
inputs will produce, it uses a kinematic (physics-based) model of the car and its actuators. Once it finds an optimal
(or at least locally optimal) trajectory, the car executes the actuations from just the first timestep of the planned
trajectory. Once that timestep has elapsed, the new ground-truth state of the car is computed from the car's sensors.
Then the process is repeated, with a new optimal trajectory being calculated based off the new ground-truth state, and
the first step of that new trajectory being executed.

