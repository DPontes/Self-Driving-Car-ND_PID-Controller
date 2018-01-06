# PID Controller Project

## Overview

This project implements a **PID Controller** algorithm in C++ to control the steering while driving a simulated vehicle around a track using feedback on the measured lateral **Cross-Track Error (CTE)**.

Besides implementing the basic **Proportional/Integral/Derivative error terms**, the PID controller includes some additional features such as an integral windup min/max guard, D term latching and smoothing to prevent spikes from CTE discontinuities, and rate limiting on final output to smooth-out the steering movement.

The PID control gains were tuned manually initially to explore their effects. After base gains were chosen at the reference speed (throttle = 0.3), further tuning adjustments were done using the **twiddle algorithm** (coordinate ascent) to optimize an error function based on combining accumulated CTE and steering work.

## Initial Manual Tuning to Explore P, I, D Component effects

PID controllers are one of the simplest types of feedback controller to implement, but can sometimes be **very difficult to tune for good stable performance in all conditions**, especially for controlling something like the position of a vehicle by its steering wheel.

It is quite difficult to control the steering angle of a vehicle with only measurements from CTE (Cross-Track Error) as this only provides error information of it's current location, without any look-ahead knowledge. Also, the controller affects the **yaw angle of the vehicle to inderectly adjust the lateral position** without any way to plan proper counter-steering to keep the vehicle's yaw aligned with the road once the CTE is minimized.

This means that tuning these PID gains will always have some **oscillation or overshoot** which will become more severe as the speed of the vehicle increases, so this kind of control application is not really well-suited for controlling the steering of a vehicle, but can be useful as an exercise to explore the P, I and D component effects in general.

### P Control

Due to the inherent instabilities of steering angle feedback from CTE mentioned above, using only **Proportional** control is **not feasible** because the vehcle will **constantly overshoot and oscillate no matter what value of P gain is used.** In order to stabilize the control, the Derivative component must also be used to do some counter-steering as the error is decreasing but before crossing over zero to reduce the lateral overshoot.

### PD Control

To tune a PD controller, I initially chose a P gain that was **strong enough to keep the CTE within the track width through the sharpest turns** of the track while the D gain was **high enough to prevent oscillations** from causing the vehicle to crash.

The observed behaviour from the vehicle with these parameters was of some steering angle spikes. These spikes are from discontinuities in the raw CTE measurements which are amplified by the Derivative gain that is applied to discrete derivative of the CTE signal. These spikes would cause unnacceptable steering behaviour; therefore, some more filtering is necessary.

### PD COntrol with Filters

To improve the control smoothness and robustness to noise, some filters were implemented:

1. D term max guard
2. D term smoothing by an exponential smoothing factor
3. PD output rate limit

The **D term max guard** was tuned based on the maginitude of the D error term's general movement in the data.

The **D term smoothing** was tuned to be as small as possible to prevent response delay that could cause instability from the control phase shift but still provide some smoothing benefit.

The **PD output rate limit** was tuned based on what seemed to be needed to allow reasonably quick enough steering movement in the sharp turns, similar to how a real driver would not turn the steering wheel faster than a certain rate.

These filters improved the noise spikes in steering and D error term while still maintaining the overall control behaviour. However, the CTE still has some steady offset from zero in the long gradual turn in the track. To reduce this offset, the **Integral** component is also needed.

### PID Control

To tune the PID controller, I kept P and D gains from the PD controller and simply started **adding I gain until CTE offset was eliminated overall** without causing additional oscillations and overshoots.

The I error term movement was steady overall and had peak magnitudes similar to the P and D terms in the sharp turns. This had the additional benefit of reducing the peak CTE from ~3m to ~2m.

The steering movement is still not really smooth as expected from the limitations of this kind of applications, but at least the **vehicle was able to drive around the track** with pretty simple straight-forward tuning of the gains. In order to see if further improvement was possible by finer tuning, the **Twiddle algorithm (Coordinate Ascent)** was also implemented.

### Twiddle Automatic Tuning to Choose the Final Parameters

 The twiddle algorithm automatically runs repeated simulations to test if adjusting the gain parameters up or down a little can improve an overall error function. Each time the error function result is improved, the gain adjustment is kept and then twiddled further with larger or smaller deltas until no further benefit is found.

 To implement this, the initial gains were set as the base PID controller gains discussed previously, and the initial deltas were set to be half of the magnitudes of each gain. The **error function** was set as the accumulation of the CTE plus the accumulation of the steering angle. This combination tries to minimize the overall CTE while also penalizing busy steering work.

 After twiddling, the P gain reduced from 0.1 to 0.084271, the I gain reduced from 0.001 to 0.000690 and the D gain increased from 2.0 to 3.0.

 Overall, the **steering is slightly more aggressive** from the the increased D gain, but the **CTE is kept a little tighter around zero**. This means that the error function still biased toward reducing CTE more than keeping smooth steering. 

## Key Files

| File              | Description                                                                                                    |
|:-----------------:|:--------------------------------------------------------------------------------------------------------------:|
| /src/main.cpp     | Source code for **main loop** that handles **uWebSockets communication to simulator**                          |
| /src/PID.cpp, .h  | Source code for **PID control algorithm** that controls the steering value based on feedback from measured CTE |
| install-mac.sh    | Script for Mac to install uWebSocketIO required to interface with simulator                                    |
| install-ubuntu.sh | Script for Linux to install uWebSocketIO required to interface with simulator                                  |

The original Udacity project repository is [here](https://github.com/udacity/CarND-PID-Control-Project).

## How to Build and Run Code

This project involves the Udacity Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two scripts (**install-mac.sh** and **install-ubuntu.sh**) that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and run by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./pid

## Other Important Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
