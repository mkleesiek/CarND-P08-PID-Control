#include "PID.h"

using namespace std;

void PID::Init(double Kp, double Ki, double Kd)
{
  // Initialize PID coefficients
  m_Kp = Kp;
  m_Ki = Ki;
  m_Kd = Kd;

  // and errors
  m_p_error = 0.0;
  m_d_error = 0.0;
  m_i_error = 0.0;
}

std::array<double, 3> PID::GetCoefficients() const
{
    return {m_Kp, m_Ki, m_Kd};
}

void PID::UpdateError(double cte)
{
  // Update PID errors based on cte
  const double cte_previous = m_p_error;
  m_p_error = cte;
  m_d_error = cte - cte_previous;
  m_i_error += cte;
}

double PID::TotalError() const
{
  // Calculate and return the total error
  return m_Kp * m_p_error + m_Kd * m_d_error + m_Ki * m_i_error;
}
