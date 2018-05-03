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

## Model Description
The kinematic model of the car tracks the car's state as a 4-element vector: [*x*, *y*, *&psi;*, *v*]. *x* and *y* are
the coordinates of the car, relative to the car (so *x* = *y* = 0 at *t* = 0, but they will be non-zero for *t* > 0).
*&psi;* is the heading of the car, relative to the car (likewise it will be zero at *t* = 0). For *x*, *y*, and *&psi;*
a coordinate system is used where the car's direction points along the positive *x* axis. *v* is the car's speed.

The model represents the actuations as a 2-element vector: [*&delta;*, *a*]. *&delta;* represents the steering input to
the car as an angle between -25&deg; and 25&deg;. *a* represents the acceleration input as a value between -1
miles/hour/second and 1 miles/hour/second. Negative values represent braking, positive values represent accelerating.

For a given timestep with an elapsed time *&Delta;t*, the following equations are used to predict the new state
[*x*<sub>*i* + 1</sub>, *y*<sub>*i* + 1</sub>, *&psi;*<sub>*i* + 1</sub>, *v*<sub>*i* + 1</sub>] from the current state
[*x*<sub>*i*</sub>, *y*<sub>*i*</sub>, *&psi;*<sub>*i*</sub>, *v*<sub>*i*</sub>]:

*x*<sub>*i* + 1</sub> = *x*<sub>*i*</sub> + *v*<sub>*i*</sub> × cos(*v*<sub>*i*</sub>) × *&Delta;t*

*y*<sub>*i* + 1</sub> = *y*<sub>*i*</sub> + *v*<sub>*i*</sub> × sin(*v*<sub>*i*</sub>) × *&Delta;t*

*&psi;*<sub>*i* + 1</sub> = *&psi;*<sub>*i*</sub> - *&delta;* × *v*<sub>*i*</sub> / *Lf* × *&Delta;t*

*v*<sub>*i* + 1</sub> = *v*<sub>*i*</sub> + *a*<sub>*i*</sub> × *&Delta;t*

In the 3rd equation above, *Lf* is a constant that determines how fast the car turns for a given steering angle. Cars
with a larger wheelbase will have higher *Lf* values.

## Choosing *N* and *&Delta;t*
When implementing a model predictive controller, one must choose values for the parameters *N* and *&Delta;t*. *N* is
the number of future timesteps that the trajectory is optimized over, and *&Delta;t* is the size of each timestep.

I chose *&Delta;t* = 100ms because it made it easy to deal with the 100ms actuation lag (actuations can simply take
effect one timestep later to simulate the lag). I chose *N* = 15 because I found that it gave results and had good
performance (in fact, higher values of *N* would cause the car to sometimes choose incorrect trajectories involving
U-turns).

## Fitting a Polynomial to the Waypoints
To define the reference trajectory, I fit a 3rd-degree polynomial to the input waypoints. To make the trajectory more
realistic and to also prevent the trajectory from "jumping" whenever the car passed a waypoint, I also added the car's
current position (the origin, since the polynomial is computed in the car's coordinate space) to the set of waypoints,
so that the reference trajectory will come close to the origin.

## Dealing with Latency
The controller simulates 100ms of actuation latency by waiting 100ms after an actuation is computed before sending it
to the simulator.

I found that at lower speeds, e.g. less than 20 mi/h, the car was still able to drive around the track even with no
special compensation for the latency. However, at speeds higher than that, the car would eventually derail.

In order to compensate for the latency, I changed my model to simulate the latency's effect. Since my *&Delta;t* is
100ms, this simply means using the actuations from timestep *i* - 1, instead of timestep *i*, to compute the transition
from timestep *i* to timestep *i + 1*. This also means that after I calculated *&delta* and *a* values, I needed to save
them and pass them into the model in the next iteration of the loop.

I was able to get the car to successfully drive around the track at over 45 mi/h.
