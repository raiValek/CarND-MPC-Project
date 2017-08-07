# CarND Model Predictive Control (MPC) 
Self-Driving Car Engineer Nanodegree Program

---

## Introduction

Like in the [PID control project](https://github.com/raiValek/CarND-PID-Control-Project) the purpose of this project is to drive a car around a track automatically in a [simulator](https://github.com/udacity/self-driving-car-sim/releases). This time a model-driven approach is used. Model Predictive Control (MPC) uses given information about the object to calculate a more accurate estimate for the actuators.

## The Model

The used model is of pure kinematic nature and discribes just the geometric movement of the vehicle. There are no forces taken into account. At each timestep `dt` the following equations compute the next position `x` and `y`, heading `psi` and velocity `v`.

    x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
    y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
    psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
    v_[t+1] = v[t] + a[t] * dt
    cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
    epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt


This is done in order to reduce a cost function that includes the cross track error `cte` and the orientation error `epsi`. Other parameters are

- `f(x[t])` the `y` value of the desired trajectory described with a polynomial at point `x` at time `t`
- `Lf` the distance of the cars front to the center of gravity
- `psides[t]` the tangential angle of the polynomial `f`

The goal is to find the optimal actuator values of the steering `delta` and the throttle `a` to follow the desired trajectory `f` as close as possible.

## Optimization

MPC needs to know a desired path to compute the right actuator values. To get the optimal actuations, the model discribed above will be optimized in order to minimize a specific cost function. The design of this cost function is crucial for the quality of the actuations. First, the path is represented by a given arbitrary number of waypoints. To get a fully connected and smooth path, the waypoints are fitted into a 3rd grade polynomial. The first two components of the cost function, cross track error `cte` and the orientation error `epsi`, are the deviation of this polynomial. But this is not enough to satisfy the need for a smooth and safe drive. To avoid oscillations in the use of the actuators, both actuation values and their changes to the last timestep are part of the costs. Finally the deviation of the velocity to a desired velocity is taken into account.

The computation of the next acutations has to go much farther then just the next timestep to follow the desired path as close as possible. The time between two steps `dt` and the number of estimated timesteps `N` are crucial parameters to choose and have a significant impact to the systems stability. The product `N * dt` gives the time the controller tries to optimize in advance. The "right" values highly depend on the desired velocity. When the car is driving fast, the controller has to know more of the next upcoming path in order to follow it smoothly. A too short prediction time can rapidly lead to oscillations. On the other hand a too short timestep `dt` in combination with a long prediction time can lead to a too long computation time. Hence there is always a tradeoff to choose a long enough prediction time with as less as possible timesteps.

The values `N = 13` and `dt = 0.07` led to a stable behaviour at a desired velocity of 75 mph.

It became apparent quickly that the change of the steering between two timesteps is a very important part of the cost function and therefore needs to be weighted higher than the other parts. The cars behaviour became only calm enough after a weight of 2500.

## Latency

In a real environment the actuators will not follow it's commands instantly. Between the command and the actual actions will always be a reaction time. Hence the car will not react at the next timestep but in `latency/dt` timesteps. This has to be part of the vehicles model to react properly. In this project the latency is set to 100 ms. To let the optimizer know that no actuation is possible within reaction time, the upper and lower bounds for each timestep within reaction time is set to the last actuation value.

    // During latency the acceleration is clamped to the last value
    for (i = delta_start; i < delta_start+latency_steps; i++) {
      vars_lowerbound[i] = last_delta;
      vars_upperbound[i] = last_delta;
    }

    // The upper and lower limits of delta are set to -25 and 25
    for (i = delta_start+latency_steps; i < a_start; i++) {
      vars_lowerbound[i] = -0.436332;
      vars_upperbound[i] = 0.436332;
    }

The first computed actuations after reaction time are used to control the vehicle.

## Result

The following video* shows some laps controlled by the described approach above.

[![](https://img.youtube.com/vi/fxAWMwrW1Ek/0.jpg)](https://www.youtube.com/watch?v=fxAWMwrW1Ek)

*Since the frame capturing spoiled the model by introducing another latency, I had to record the monitors output with a camera. Sorry for potato quality.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.