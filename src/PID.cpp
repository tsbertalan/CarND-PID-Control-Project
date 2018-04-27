#include <iostream>
#include "PID.h"

/*
 * How long should the deque of error values be when computing the i term?
 *
 * 200 samples (About 49 [ms] each) is roughly enough time
 * to judge manually whether we're turning,
 * and so probably enough time to keep track of integrated history.
 */
#define MAX_CTE_HISTORY_LENGTH 200

using namespace std;

/*
 * @brief       Construct PID controller.
 * All coefficients are set to 0 by default.
 */
PID::PID() {
    p_error = 0;
    i_error = 0;
    d_error = 0;
    last_t = epoch_time();
}

PID::~PID() {}

/*
 * @brief       Set PID coefficients.
 * @param[in]   Kp, Ki, Kd      the values of the coefficients to set
 */
void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

/*
 * @brief       Do the actual PID computations.
 * @param[in]   cte             the error signal to be driven to zero
 */
void PID::UpdateError(double cte) {

    // dt is often about 49, so this factor is often 1. However, if it varies,
    // this might help the given parameters generalize better.
    long t = epoch_time();
    double dt = ((double (t)) - last_t) / 49.0;
    last_t = t;

    cte_history.push_back(cte);
    d_error = (cte - p_error) / dt;
    p_error = cte;
    i_error += cte * dt;

    if(cte_history.size() >= MAX_CTE_HISTORY_LENGTH) {
        i_error -= cte_history[0];
        cte_history.pop_front();
    }

}

/*
 * @brief       Get the summed error terms, weighted by their coefficients.
 * This can be used as the feedback value.
 * @return      The summed error.
 */
double PID::TotalError() {
    return - Kp * p_error - Ki * i_error - Kd * d_error;
}

