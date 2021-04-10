#pragma once

#include <array>

/**
 * A simple proportional–integral–derivative (PID) controller class.
 */
class PID {
public:
  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

  /**
   * Retrieve currently configured coefficients.
   * @return An array of length 3.
   */
  std::array<double, 3> GetCoefficients() const;

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError() const;

private:
  /**
   * PID Errors
   */
  double m_p_error = 0.0;
  double m_i_error = 0.0;
  double m_d_error = 0.0;

  /**
   * PID Coefficients
   */ 
  double m_Kp = 0.0;
  double m_Ki = 0.0;
  double m_Kd = 0.0;
};

inline void PID::Init(double Kp, double Ki, double Kd)
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

inline std::array<double, 3> PID::GetCoefficients() const
{
    return {m_Kp, m_Ki, m_Kd};
}

inline void PID::UpdateError(double cte)
{
  // Update PID errors based on cte
  const double cte_previous = m_p_error;
  m_p_error = cte;
  m_d_error = cte - cte_previous;
  m_i_error += cte;
}

inline double PID::TotalError() const
{
  // Calculate and return the total error
  return m_Kp * m_p_error + m_Kd * m_d_error + m_Ki * m_i_error;
}