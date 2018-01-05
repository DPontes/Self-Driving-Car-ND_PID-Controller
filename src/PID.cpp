#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

/*
  PID Reset to clear error terms for each drive
*/

void PID::Reset(){
  // Initialize error terms
  p_error_          = 0.0;
  i_error_          = 0.0;
  d_error_          = 0.0;

  // Initialize stored previous values
  prev_d_error_     = 0.0;
  prev_total_error_ = 0.0;

  // Initialize twiddle error term
  twiddle_error_    = 0.0;
} //PID::Reset()

/*
  PID initialization to set gains and other stored parameters.
  Reset function is also called to clear error terms
*/
void PID::Init(double Kp, double Ki, double Kd,
               double i_max. double d_smooth, double error_rate_max) {
  // Initialize parameters
  Kp_             = Kp;
  Ki_             = Ki;
  Kd_             = Kd;
  i_max_          = i_max;
  i_cut_          = false;
  d_max_          = dmax;
  d_smooth_       = d_smooth;
  error_rate_max_ = error_rate_max;

  // Initialize twiddle parameters
  Kgains_ = {&Kp_, &Ki_, &Kd_};
  Kdeltas_ = {0.05, 0.0005, 1.0};
  twiddle_best_error_ = __DBL_MAX__;
  twiddle_idx_ = 0;
  twiddle_switch_ = false;

  // Reset PID terms to initial values
  Reset();
} // PID::Init()

/*

*/
void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}
