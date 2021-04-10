/**
 * @author marco.kleesiek@gmail.com
 */

#include <cstdlib>
#include <iostream>
#include "driver.h"
#include "optimizer.h"

using namespace std;

int main()
{
  Driver driver{};
  // Setup the "driver" with the number of iterations representing roughly 1 lap on the simulator track.
  // Passing a value of "0" will let the driver run indefinitely without resetting the simulator.
  driver.SetMaxIterations(900);

  // Set initial PID coefficient values
  driver.ConfigureSteeringPID(0.18, 0.0001, 1.5);
  driver.ConfigureThrottlePID(3.15623, 0.0001, 0.110816);

  /*
   * If you are not interested in optimizing the PID coefficients, simply execute ::Run()
   */

//  driver.Run();
//
//  return EXIT_SUCCESS;

  /*
   * Down below find the procedure for executing the minimizer:
   */

  // setup optimizer
  TwiddleOptimizer<Driver> optimizer{driver};

  // specify initial parameter values and step sizes for PID arguments
  // index 0, 1, 2: steering Kp, Ki, Kd
  // index 3, 4, 5: throttle Kp, Ki, Kd
  std::vector<double> pid_params{0.18, 0.0001, 1.9, 3.0, 0.0001, 5.0};
  std::vector<double> pid_deltas{0.01, 0.00005, 0.1, 0.1, 0.00005, 0.1};

  // minimize until the sum of deltas as fallen below a certain threshold
  bool success = optimizer.Minimize(pid_params, pid_deltas, 0.002);

  if (success)
  {
    cout << "Optimizer converged!" << endl
         << "Steering: Kp = " << pid_params[0] << ", Ki = " << pid_params[1] << ", Kd = " << pid_params[2] << endl
         << "Throttle: Kp = " << pid_params[3] << ", Ki = " << pid_params[4] << ", Kd = " << pid_params[5] << endl;
    return EXIT_SUCCESS;
  }
  else
  {
    return EXIT_FAILURE;
  }
}
