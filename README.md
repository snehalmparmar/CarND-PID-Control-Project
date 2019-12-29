# PID Controller

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

![PIDController_Output](https://github.com/snehalmparmar/CarND-PID-Control-Project/blob/master/src/video_output/2019-12-29-14-21-02_GIF.gif)

The PID uses the cross-track error (CTE) to correct the steering angle. As a result, the vehicle will stay on the road.

The steering angle SA of the vehicle is set by SA = Kp x CTE + Ki x int(CTE x dt) + Kd d(CTE)/dt. 

This is the sum of three terms : proportional, integral and differential. 

**Proportional term.** The goal of the proportional term is to make the vehicle steer towards the line of zero-CTE. The proportional term is the product of cross-track error by a coefficient (Kp). This will tend to reduce the CTE but - if used alone without the integral and differential terms - the vehicle will steer towards the zero CTE line, then when it crosses this line it will have a 0 degrees steering angle (SA = Kp x CTE = 0) but a non-zero yaw angle w.r.t. the path it is trying to follow (the line where CTE = 0), and will cross this line (overshoot). 

**Differential term.** The goal of the differntial term is to make the yaw angle approach zero as the vehicle closes to the center line (where CTE=0).  It is the product of the difference in error between the current CTE and the previous CTE by a coefficient (Kp). If there was an increase in error, we will increase the steering correction. If the error decreased, we will decrease steering correction proportionally to the amount it decreased. This insures the vehicle will converge towards the zero error (the path it is trying to follow) Integral term. 

**Integral term.** The goal of the integral term is to cancel-out a systematic error or bias. It is the product of the cumulative error by a coefficient (Ki). The cumulative error is the sum of the errors for every iteration since the beginning of the simulation. If the yaw rate of the vehicle is non-zero for a desired angle of zero, there is a systematic error (the actual angle of the wheels is different than zero when the sensor reads it is zero, for example). In this case, the vehicle will always drive at an offset if P and D terms are tuned perfectly. By taking the integral of the CTE over time and applying a proportional correction, this systematic error will end up being cancelled. 

The PID requires tuning the Kp, Ki and Kd terms. This tuning was achieved using Twiddle algorithm explained in class. The twiddle algorithm adds a small value to a parameter (Kp, Ki or Kd), than calculates the effect of this tweak on the CTE over the course of a given number of iterations. Depending on the behaviour of the error it will decrease the parameter, reset the change to zero or maintain the change it made. A drawback of this implementation is that the integral error is never reset, therefore the effect of adjusting the Ki coefficient by a given amount is different at each iteration. Ideally, the integral error should be reinitialized between runs of twiddle. 

It can be seen in the following video that with current initial parameters : Kp = 0.1, Ki = 0, Kd = 0.5 the PID already does good. The vehicle oscillates slightly (overshoots), suggesting that the Kd parameter could be higher. The error is reduced by tweaking Kp higher but is not reduced by tweaking neither Ki nor Kd, suggesting that smaller tweaking steps may be required. If let run long enough, the car will run out of the road because the integral error will grow faster than the tweaking steps on Ki are decreasing. To overcome that, it might be necessary to reset the integral error at every twiddle iteration. 

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)
