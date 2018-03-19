#ifndef PID_H
#define PID_H
#include <iostream>
#include <vector>

class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;

  bool isTwiddle;  // whether to apply twiddle or not
  int step;        // count the number of steps
  int max_step;    // maximum number of stpes before restart
  const int buffer_step = 5;  
  double err;       // stores the error
  double best_error;  // best error for twiddle
  std::vector<double> dp; // vector of possible changes to the coefficients

  int n_gain;   // indicate which gain to tune
  int n_tune;   // indicate the time of tuning

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, bool tune=false);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  std::vector<double> doTwiddle();
};

#endif /* PID_H */
