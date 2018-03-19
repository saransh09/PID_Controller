#include "PID.h"
#include <float.h>
#include <limits>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
	best_error = DBL_MAX;  // initialize the best_error as maximum possible
	dp.push_back(0.1);	   // initialize the values of Kp
	dp.push_back(0.005);   // initialize the values of Ki
	dp.push_back(0.1);     // initialize the values of Kd
	n_gain = 0;			   // start from the first parameter to tune
	n_tune = 1;			   // time of tuning is 1
	max_step = 25;		   // tune at most for 25 timesteps
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, bool tune) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;
	// twiddle parameters
	isTwiddle = tune;
	step = 0;
	err = 0;
	return;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
	++step;
	if(step > max_step && isTwiddle){
		err += cte*cte;
	}
	return;
}

double PID::TotalError() {
	double total = -Kp*p_error - Ki*i_error - Kd*d_error;
	return total; 
}

std::vector<double> PID::doTwiddle(){
	std::vector<double> gain{Kp,Ki,Kd};
	if(best_error == DBL_MAX){
		best_error = err;
		err = 0;
		n_gain = 0;
		n_tune = 0;
	}
	while(dp[0]+dp[1]+dp[2] > 0.02){
		if(n_tune==0){
			gain[n_gain] += dp[n_gain];
			err = 0;
			n_tune = 1;
			return gain;
		}
		else if(n_tune==1){
			if(err < best_error){
				best_error = err;
				dp[n_gain] *= 1.1;
				err = 0;
				n_tune = 0;
				n_gain = (n_gain+1)%3;
			}
			else{
				gain[n_gain] -= 2*dp[n_gain];
				err = 0;
				n_tune = 2;
				return gain;
			}
		}
		else if(n_tune==2){
			if(err < best_error){
				best_error = err;
				dp[n_gain] *= 1.1;
				err = 0;
				n_tune = 0;
				n_gain = (n_gain+1)%3;
			}
			else{
				gain[n_gain] += dp[n_gain];
				dp[n_gain] *= 0.9;
				err = 0;
				n_tune = 0;
				n_gain = (n_gain+1)%3;
			}
		}
	}
	std::cout<<"----Finishing Tuning-----"<<std::endl;
	return gain;
}
