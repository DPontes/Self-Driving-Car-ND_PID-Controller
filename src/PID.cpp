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
void PID::Init(double Kp, double Ki, double Kd) {

}

void PID::UpdateError(double cte) {
}

double PID::TotalError() {
}
