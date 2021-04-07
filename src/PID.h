#pragma once

#include <array>

class PID {
public:
  /**
   * Constructor
   */
  PID() = default;

  /**
   * Destructor.
   */
  virtual ~PID() = default;

  /**
   * Initialize PID.
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

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
