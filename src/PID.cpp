#include "PID.h"
#include <cmath>
#include <algorithm>

void PID::Init(double Kp, double Ki, double Kd) {
  // Initialize PID coefficients
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;

  // and errors
  m_p_error = 0.0;
  m_d_error = 0.0;
  m_i_error = 0.0;

  // and deltas
  constexpr double fraction = 0.1;
  m_delta_Kp = fraction * m_Kp;
  m_delta_Ki = fraction * m_Ki;
  m_delta_Kd = fraction * m_Kd;
}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte
  const double cte_previous = m_p_error;
  m_p_error = cte;
  m_d_error = cte - cte_previous;
  m_i_error += cte;

  m_iteration++;
}

double PID::TotalError() const {
  // Calculate and return the total error
  return m_Kp * m_p_error + m_Kd * m_d_error + m_Ki * m_i_error;
}

double PID::Output(double min, double max) const {
  double output = -TotalError();
  output = std::min(output, max);
  output = std::max(output, min);
  return output;
//  return (max - min) / (1.0 + exp(total_error)) + min;
//  return (max - min) * tanh(-total_error) + min;
}