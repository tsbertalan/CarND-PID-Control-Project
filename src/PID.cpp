#include <iostream>
#include "PID.h"

// 200 samples (About 49 [ms] each) is enough time to judge whether we're turning,
// and so enough time to keep track of integrated history.
#define MAX_CTE_HISTORY_LENGTH 200

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
    p_error = 0;
    i_error = 0;
    d_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void PID::UpdateError(double cte) {
    cte_history.push_back(cte);
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    if(cte_history.size() >= MAX_CTE_HISTORY_LENGTH) {
        i_error -= cte_history[0];
        cte_history.pop_front();
    }

}

double PID::TotalError() {
    return - Kp * p_error - Ki * i_error - Kd * d_error;
}

