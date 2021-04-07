#pragma once

#include "PID.h"
#include <uWS/uWS.h>

class Runner
{
public:
  enum class Mode {
    Steering,
    Throttle,
    Both
  };

  Runner() = default;
  Runner(const Runner& other) = delete;
  virtual ~Runner() = default;

  void ConfigureSteeringPID(double Kp = 0.1, double Ki = 0.0001, double Kd = 0.7);
  void ConfigureThrottlePID(double Kp = 0.1, double Ki = 0.0001, double Kd = 0.7);
  void SetMaxIterations(size_t max_iterations);

  double Run();

  void SetOptimizingMode(Mode mode = Mode::Steering);
  double operator() (const std::vector<double>& params);

private:
  void ConfigureCallbacks();

  uWS::Hub m_hub;
  int m_port = 4567;

  PID m_pid_steering;
  PID m_pid_throttle;

  size_t m_iteration = 0;
  size_t m_max_iterations = 0;

  Mode m_mode = Mode::Steering;
  double m_total_sq_error = 0.0;
};
