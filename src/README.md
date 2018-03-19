# PID_Controller

This is a very basic implementation of a PID Controller, with an option to tune the hyperparameters of using Twiddle.
The PID Controller uses CTE (Cross track error) to penalize the change in any particular quantity. In our application, we are using it to steer a car, so that, it can go through a track, without leaving the track. 

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
