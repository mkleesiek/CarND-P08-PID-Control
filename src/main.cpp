#include <cstdlib>
#include "runner.h"
#include "optimizer.h"

int main()
{
  Runner runner{};
  runner.SetMaxIterations(500);
  runner.ConfigureSteeringPID(0.08, 0.0001, 0.6);
  runner.ConfigureThrottlePID(0.5, 0.0001, 0.1);
//  runner.Run();
//
//  return EXIT_SUCCESS;

  runner.SetOptimizingMode(Runner::Mode::Throttle);
  TwiddleOptimizer<Runner> optimizer{runner};
//  std::vector<double> pid_params{0.08, 0.0001, 0.6};
  std::vector<double> pid_params{0.5, 0.0001, 0.1};
  bool success = optimizer.Minimize(pid_params);

  return (success) ? EXIT_SUCCESS : EXIT_FAILURE;
}
