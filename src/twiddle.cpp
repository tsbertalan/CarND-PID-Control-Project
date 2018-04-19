#include "twiddle.h"
#include <iostream>
#include <cmath>

using namespace std;

constexpr double DEFAULT_DIFF_PARAMS = 0.01;


template <typename T>
void printvec(vector<T> v) {
	cout << "[";
	for(auto & x : v) {
		cout << x << ", ";
	}
	cout << "]";
}

template <typename T>
void printvec(vector<T> v, string name) {
	cout << name << " = ";
	printvec(v);
	cout << endl;
}


Twiddler::Twiddler(int nparams=6, double tol=0.0001) {
	vector<double> parameters(nparams, 0);
	vector<double> diff_parameters(nparams, DEFAULT_DIFF_PARAMS);

	cout << "Initialized Twiddler with" << endl;
	printvec(parameters, "p");
	printvec(diff_parameters, "dp");

	i_param = 0;
	iterations = 0;
	this->tol = tol;
	best_error = std::numeric_limits<double>::infinity();
	last_change = NONE;

	declared_convergence = false;
}


bool Twiddler::twiddle(double error) {

	// Save the error history.
	errors.push_back(error);

	// If we previously declared convergence, just exit.
	if(declared_convergence)
		return true;

	// If we've finished the last parameter loop, check for convergence.
	if(i_param == parameters.size()) {

		// Increment the count-of-loops.
		iterations++;

		cout << endl << "=====================" << endl;
		cout << "Twiddle iteration " << iterations << ":" << endl;

		// Sum the increment vector.
		double sdp = 0;
	    for(auto& d : diff_parameters) {
	    	sdp += d;
	    }

	    // Check for convergence.
	    if(sdp <= tol) {
	    	// If we've converged, just say so.
	    	cout << "Converged! sum(dp)=" << sdp << " <= tol=" << tol << endl;
	    	declared_convergence = true;
	    	return true;
	    } else {
	    	// If we haven't converged, reset the counter for the next loop.
	    	cout << "Not yet converged. sum(dp)=" << sdp << " > tol=" << tol << endl;
			i_param = 0;
	    }
	}

	// If we haven't converged, or we're in the middle of a loop,
	// we now do the actual twiddle.

	// Depending on what we did last time an error was passed, 
	// we can now do one of three things.

	// We haven't tried this dp yet.
	// Try an increase.
	if(last_change == NONE) {
		cout << endl << "Try increasing p[" << i_param << "]." << endl;
		parameters[i_param] += diff_parameters[i_param];
		printvec(parameters, "p");
		last_change = INCREASE;

	// We did an increase last time; how did it work out?
	} else if(last_change == INCREASE) {

		// If it succeeded, accelerate.
		if(error < best_error) {
			cout << "Increase succeeded!" << endl;
			succeed(error);

		// If it failed, try a decrease.
		} else {
			cout << "Increase failed!" << endl;
			cout << endl << "Try decreasing p[" << i_param << "]." << endl;
			parameters[i_param] -= 2 * diff_parameters[i_param];
			printvec(parameters, "p");
			last_change = DECREASE;
		}

	// We did a decrease last time; how did it work out?
	} else if(last_change == DECREASE) {
		
		// If it succeeded, accelerate.
		if(error < best_error) {
			cout << "Decrease succeeded!" << endl;
			succeed(error);

		} else {
			// If the decrease also failed, restore the original parameter value and decelerate.
			cout << "Decrease failed!" << endl;
			parameters[i_param] += diff_parameters[i_param];
			printvec(parameters, "p");
			fail();
		}
	}

	return false;
}

void Twiddler::succeed(double error) {
	cout << "Recording new best error of " << error;
	cout << " and increasing dp[" << i_param << "]." << endl;
	diff_parameters[i_param] *= 1.1;
	printvec(diff_parameters, "dp");
	best_error = error;
	cout << "Best error decreased to " << best_error << "." << endl;
	moveOn();
}

void Twiddler::fail() {
	cout << endl << "Failed twiddle." << endl;
	cout << "Decreasing dp[" << i_param << "]" << endl;
	diff_parameters[i_param] *= 0.9;
	printvec(diff_parameters, "dp");
	moveOn();
}

void Twiddler::moveOn() {
	// Move on to next parameter.
	cout << "Moving on from parameter " << i_param << "." << endl;
	i_param++;
	last_change = NONE;
}


std::vector<double> Twiddler::get_params() {
	return parameters;
}

void Twiddler::set_params(vector<double> new_parameters) {
	cout << "Changed p from" << endl;
	printvec(parameters, "p");
	parameters = new_parameters;
	cout << "to" << endl;
	printvec(parameters, "p");
}


void Twiddler::set_diff_params(vector<double> new_diff_parameters) {
	cout << "Changed dp from" << endl;
	printvec(diff_parameters, "dp");
	diff_parameters = new_diff_parameters;
	cout << "to" << endl;
	printvec(diff_parameters, "dp");
}

bool Twiddler::is_converged() {
	return declared_convergence;
}


TwiddlerManager::TwiddlerManager(std::vector<PID*>& pids, int tmax, double tol) {
	this->pids = pids;
	// for(auto &pid : pids) {
	// 	this->pids.push_back(pid);
	// }
	this->tmax = tmax;

	int nparams = pids.size() * 3;

	twiddler = Twiddler(nparams, tol);

	// Extract the existing parameters.
	vector<double> new_parameters(nparams);
	vector<double> new_diff_parameters(nparams);
	int i = 0;
	for(auto &pid : pids) {
		new_parameters[i*3+0] = pid->Kp;
		new_parameters[i*3+1] = pid->Ki;
		new_parameters[i*3+2] = pid->Kd;

		new_diff_parameters[i*3+0] = fabs(pid->Kp * .1);
		new_diff_parameters[i*3+1] = fabs(pid->Ki * .1);
		new_diff_parameters[i*3+2] = fabs(pid->Kd * .1);
		
		i++;
	}
	twiddler.set_params(new_parameters);
	twiddler.set_diff_params(new_diff_parameters);

	errors_seen = 0;
	integrated_error = 0;
}

void TwiddlerManager::process_error(double error) {

	// Integrate the error.
	integrated_error += error;
	errors_seen++;

	// If we've added up enough errors, take a mean.
	if(errors_seen >= tmax) {
		// cout << endl;

		integrated_error /= errors_seen;

		if(!twiddler.is_converged())
			cout << "Mean absolute error on run: " << integrated_error << endl;

		// Then twiddle the parameters.
		twiddler.twiddle(integrated_error);

		// Apply the new parameters to the PIDs.
		int i = 0;
		vector<double> p = twiddler.get_params();
		for(auto &pid : pids) {
			pid->Init(fabs(p[i*3]), fabs(p[i*3+1]), fabs(p[i*3+2]));
			i++;
		}

		// Reset the count.
		integrated_error = 0;
		errors_seen = 0;
	// } else {
	// 	cout << integrated_error << " ";
	}
}