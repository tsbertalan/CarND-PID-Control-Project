#ifndef TWIDDLE_H
#define TWIDDLE_H

#include <vector>
#include <limits>
#include "PID.h"

enum last_change_enum { INCREASE, DECREASE, NONE };
typedef enum last_change_enum last_change_t;


class Twiddler {

private:
  std::vector<double> parameters;
  std::vector<double> diff_parameters;
  std::vector<double> errors;

  unsigned int i_param;
  int iterations;
  double tol;
  double best_error;
  last_change_t last_change;

public:
  Twiddler(int nparams, double tol);
  bool twiddle(double error);
  std::vector<double> get_params();
  void set_params(std::vector<double> new_parameters);
  void set_diff_params(std::vector<double> new_diff_params);
};

class TwiddlerManager {

private:
  std::vector<PID*> pids;
  Twiddler twiddler;
  
  double integrated_error;
  int errors_seen;
  int tmax;

public:
  TwiddlerManager(std::vector<PID*>& pids, int tmax);
  void process_error(double error);

};


#endif /* TWIDDLE_H */
