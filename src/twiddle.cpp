#include "twiddle.h"
#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm> //  min, max

using namespace std;

constexpr double DEFAULT_DIFF_PARAMS = 0.01;





Twiddler::Twiddler(int nparams=6, double tol=0.0001) {
    vector<double> parameters(nparams, 0);
    vector<double> diff_parameters(nparams, DEFAULT_DIFF_PARAMS);

    cout << "Initialized Twiddler with" << endl;
    vec_print(parameters, "p");
    vec_print(diff_parameters, "dp");

    i_param = 0;
    iterations = 0;
    this->tol = tol;
    best_error = std::numeric_limits<double>::infinity();
    last_change = NONE;

    declared_convergence = false;
}


bool Twiddler::twiddle(double error) {

    // If we previously declared convergence, just exit.
    if(declared_convergence)
        return true;

    // If we've finished the last parameter loop, check for convergence.
    if(i_param == parameters.size()) {

        // Increment the count-of-loops.
        iterations++;

        cout << endl; say_time(); cout << "=============Twiddle iteration " << iterations << "==============" << endl;

        // Sum the increment vector.
        double sdp = 0;
        for(auto& d : diff_parameters) {
            sdp += d;
        }

        // Check for convergence.
        say_time();
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
        cout << endl; say_time(); cout << "----------------------i=" << i_param << "----------------------" << endl;
        say_time(); cout << "Try increasing p[" << i_param << "]." << endl;
        parameters[i_param] += diff_parameters[i_param];
        say_time(); vec_print(parameters, "p");
        last_change = INCREASE;

    // We did an increase last time; how did it work out?
    } else if(last_change == INCREASE) {

        // If it succeeded, accelerate.
        say_time();
        if(check_error(error)) {
            cout << "; increase of p[" << i_param << "] succeeded!" << endl;
            succeed(error);

        // If it failed, try a decrease.
        } else {
            cout << "; increase failed!" << endl << endl;
            say_time(); cout << "Try decreasing p[" << i_param << "]." << endl;
            parameters[i_param] -= 2 * diff_parameters[i_param];
            say_time(); vec_print(parameters, "p");
            last_change = DECREASE;
        }

    // We did a decrease last time; how did it work out?
    } else if(last_change == DECREASE) {
        
        // If it succeeded, accelerate.
        say_time();
        if(check_error(error)) {
            cout << "; decrease of p[" << i_param << "] succeeded!" << endl;
            succeed(error);

        } else {
            // If the decrease also failed, restore the original parameter value and decelerate.
            cout << "; decrease failed!" << endl;
            parameters[i_param] += diff_parameters[i_param];
            say_time(); vec_print(parameters, "p");
            fail(error);
        }
    }

    return false;
}


bool Twiddler::check_error(double error) {
    bool result = error < best_error;
    cout << "err=" << error;
    if(result) {
        cout << " < ";
    } else {
        cout << " >= ";
    }
    cout << "best=" << best_error;
    return result;
}


void Twiddler::succeed(double error) {
    say_time(); cout << "Recording new best error of " << error;
    cout << " and increasing dp[" << i_param << "]." << endl;
    diff_parameters[i_param] *= 1.5;
    say_time(); vec_print(diff_parameters, "dp");
    best_error = error;
    say_time(); cout << "Best error decreased to " << best_error << "." << endl;
    moveOn(error);
}


void Twiddler::fail(double error) {
    cout << endl; say_time(); cout << "Failed twiddle." << endl;
    say_time(); cout << "Decreasing dp[" << i_param << "]" << endl;
    diff_parameters[i_param] /= 1.5;
    say_time(); vec_print(diff_parameters, "dp");
    moveOn(error);
}


void Twiddler::moveOn(double error) {
    // Move on to next parameter.
    say_time(); cout << "Moving on from parameter " << i_param << "." << endl;
    i_param++;
    last_change = NONE;

    // Move directly on to the next parameter manipulation.
    twiddle(error);
}


std::vector<double> Twiddler::get_params() {
    return parameters;
}


void Twiddler::set_params(vector<double> new_parameters) {
    cout << "Changed p from" << endl;
    vec_print(parameters, "p");
    parameters = new_parameters;
    cout << "to" << endl;
    vec_print(parameters, "p");
}


void Twiddler::set_diff_params(vector<double> new_diff_parameters) {
    cout << "Changed dp from" << endl;
    vec_print(diff_parameters, "dp");
    diff_parameters = new_diff_parameters;
    cout << "to" << endl;
    vec_print(diff_parameters, "dp");
}


bool Twiddler::is_converged() {
    return declared_convergence;
}


TwiddlerManager::TwiddlerManager(std::vector<PID*>& pids, unsigned int tmax, double tol, unsigned int tmin=0) {
    this->pids = pids;
    this->tmax = tmax;
    this->tmin = tmin;

    num_discarded = 0;

    lambda_mean = 2.0;
    lambda_stdd = 1.0;

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

        // At most, allow the first step to be to zero,
        // not negative!
        new_diff_parameters[i*3+0] = min(0.01, pid->Kp);
        new_diff_parameters[i*3+1] = min(0.0001, pid->Ki);
        new_diff_parameters[i*3+2] = min(0.1, pid->Kd);
        
        i++;
    }
    twiddler.set_params(new_parameters);
    twiddler.set_diff_params(new_diff_parameters);
}


void TwiddlerManager::process_error(double error) {

    // Save the error history.

    if(num_discarded >= tmin) {
        errors.push_back(error);
        absolute_errors.push_back(fabs(error));
    } else {
        num_discarded++;
    }

    // If we've added up enough errors, take a mean.
    if(errors.size() >= tmax - tmin) {

        double mae = vec_mean(absolute_errors);
        double sae = vec_stdd(absolute_errors, mae);

        double me  = vec_mean(errors);
        double se  = vec_stdd(errors, me);

        double objective = lambda_mean * mae + lambda_stdd * se;

        if(!twiddler.is_converged()) {
            say_time(); cout << "Run stats (" << errors.size() << " samples):" << endl;
            say_time(); cout << "   >Mean absolute error = " << mae << endl;
            say_time(); cout << "    Stdd absolute error = " << sae << endl;
            say_time(); cout << "    Mean error          = " << me << endl;
            say_time(); cout << "   >Stdd error          = " << se << endl;
            say_time(); cout << "   ==> objective = " << objective << endl;
        }

        // Then twiddle the parameters.
        twiddler.twiddle(objective);

        // Apply the new parameters to the PIDs.
        int i = 0;
        vector<double> p = twiddler.get_params();
        for(auto &pid : pids) {
            pid->Init(fabs(p[i*3]), fabs(p[i*3+1]), fabs(p[i*3+2]));
            i++;
        }

        // Clear the run.
        absolute_errors.clear();
        errors.clear();

    }
}
