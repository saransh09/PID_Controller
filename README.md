# PID_Controller

This is a very basic implementation of a PID Controller, with an option to tune the hyperparameters of using Twiddle.
The PID Controller uses CTE (Cross track error) to penalize the change in any particular quantity. In our application, we are using it to steer a car, so that, it can go through a track, without leaving the track. 


# Explanation of the P,I and D terms

1) <b>P: </b> This is the propotionality term, which takes into account the directly observable behaviour of he car, that is it makes the car steer solely based on the current cross track error, if the car is left f the lane center, it steers towards the right, and if the car is towards the right of the lane center it steers towards the left

2) <b>I: </b> The integral term acts as a bias, which is used to reduce the systematic bias in the measurement of the cross track error, this is extremely useful towards the curves of the road, as during that time, the car tends to misbehave when only used with the Proportion and the Differential term

3) <b>D: </b> The differential part takes into account the rate of change of the cross track error, this is particularly very helpful t check the overshoot of the steering angle, caused by the P term. This is specifically true, whenever there is a turn or the road curves. This also helps the car, reach he center of the lane more smoothly.

# Explanation

In this code, I have used some distinctive features, apart from what were actually taught in the lessons

1. PID.h :-
  
  added variables: 
  isTwiddle - bool variable for whether to use twiddle or not
  step - to count the number of steps
  max_step - max number of steps befor reset
  err - track the error, for tuning purpose
  best_error - best_error for doTwiddle() function
  vector dp - keeps the track of change in parameters
  n_gain - keeps track of which coefficient to perform change on 
  
  added functions:-
  doTwiddle() - performs twiddle and then returns a vector of appropriate values of the coefficients

2. PID.cpp :-
  
  PID::Init() - all the values of the variables defined in the ehader file are appropriately initialized
  
  PID::UpdateError() - Updates the errors with respect to the CTE values
  
  PID::TotalError() - calculates the total error based on the PID equation
  
  PID::doTwiddle() - twiddle algorithm is implemented, based on the code taught by Sebastian, here n_tune is used to track the type of update performed
  
3. main.cpp :-
  
  > modified the main function to accept arguments from the terminal
  
  > added a check to avoid overshoot of the steer_value by restircting it within -1 and 1
  
  > added a check, where throttle was adjusted based on the angle and the speed of the car

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


## Getting Started

Clone the repository on your local machine, and then delete the build folder.
Then open the terminal and enter the following command:
- mkdir build && cd build && cmake .. && make

Then to run the code, use the following command, in the mentioned order
- ./pid <initial value for Kp> <Initial value for Ki> <Initial Value for Kd> <1 or 0 for whether you want to twiddle>
  OR
- ./pid
  to run the code, with some predefined values of the coefficients and no twiddling

End with an example of getting some data out of the system or using it for a little demo

## Running the tests

After the port has opened, run your Udacity Term 2 Simulator, and select the Project 3 : PID Controller
Then you can track all the values, and actually watch your PID Controller in action

## Acknowledgments

* The twiddle code has been influenced by RobinCPC, from the forums
* The Q/A video of the project

# Reflection

* The Controller was tested without D and I initially, car was not at all able to steer appropriately

* With D and I, more stabiltity was brought to the car

* Restricting the steering angle, helper in some random overshoots

* Defining values for throttle based on the angle and spees, helper in the motion in cases of crisp turns

* twiddle helps with the parameter tuning

* Kp = 1.30, Ki = 0.01 and Kd = 6.02


# Some concerns

* Even after twiddle, the car was oscillating heavily

* Need some better methods for parmeter tuning
