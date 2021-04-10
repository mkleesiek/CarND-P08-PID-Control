/**
 * @author marco.kleesiek@gmail.com
 */

#pragma once

#include "PID.h"
#include <uWS/uWS.h>

/**
 * The "driver" class starts a WebSocket server and manages communication with the Udacity simulator.
 * It used holds PID instances, which provide steering and throttle control values to the simulator, based on
 * the cross track error information given by the simulator.
 */
class Driver
{
public:
  /**
   * Default constructor.
   *
   * @param port TCP port to listen for WebSocket connections from the simulator.
   */
  Driver(int port = 4567);

  /**
   * Default destructor.
   */
  virtual ~Driver() = default;

  // Allow default move construction and assignment
  Driver(Driver&& other) = default;
  Driver& operator=(Driver&& other) = default;

  // Prevent copy construction and assignment
  // (a uWS::Hub instance cannot be copied, since it would occupy the same TCP port)
  Driver(const Driver& other) = delete;
  Driver& operator=(const Driver& other) = delete;

  /**
   * Initialize the PID controller coefficients for steering.
   */
  void ConfigureSteeringPID(double Kp, double Ki, double Kd);

  /**
   * Initialize the PID controller coefficients for throttle.
   */
  void ConfigureThrottlePID(double Kp, double Ki, double Kd);

  /**
   * Set the maximum number of communication rounds before closing the connection to the simulator.
   * @param max_iterations ::Run will return after this amount of messages exchanged with the simulator.
   * 0 means indefinitely.
   */
  void SetMaxIterations(size_t max_iterations);

  /**
   * Start the WebSocket server and provide steering/throttle control data to a simulator,
   * once connected.
   * This function takes care of updating the PID errors.
   *
   * @return The function will return eventually, if 'max_iterations' has been set > 0.
   * The returned value is the accumulated cross track error squared divided by the speed.
   */
  double Run();

  /**
   * Functor style execution of ::Run, suited for use with a function minimizer.
   * @param params These parameters are passed to the PID controllers for initialization:
   * Index 0, 1, 2: steering Kp, Ki, Kd
   * Index 3, 4, 5: throttle Kp, Ki, Kd
   * @return The accumulated cross track error squared divided by the speed.
   */
  double operator() (const std::vector<double>& params);

private:
  /**
   * Setup message callbacks for the WebSocket server.
   */
  void ConfigureCallbacks();

  /**
   * Translate the steering PID errors into steering control values.
   */
  double GetSteeringValue() const;

  /**
   * Translate the throttle PID errors into throttle control values.
   */
  double GetThrottleValue(double steering_angle, double speed) const;

  uWS::Hub m_hub;
  int m_port = 0;

  PID m_pid_steering;
  PID m_pid_throttle;

  size_t m_iteration = 0;
  size_t m_max_iterations = 0;

  // accumulated error since m_iteration == 0
  double m_acc_error = 0.0;
};
