#include "PID.h"
#include <cmath>

double PID::SigmoidFunc(double value)
{
//  return 2.0 / (1.0 + exp(-value)) - 1.0;
  return tanh(value);
}

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize PID coefficients
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;

  // and errors
  m_p_error = 0.0;
  m_d_error = 0.0;
  m_i_error = 0.0;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte
  const double cte_previous = m_p_error;
  m_p_error = cte;
  m_d_error = cte - cte_previous;
  m_i_error += cte;
}

double PID::TotalError() const {
  // Calculate and return the total error
  return m_Kp * m_p_error + m_Kd * m_d_error + m_Ki * m_i_error;
}