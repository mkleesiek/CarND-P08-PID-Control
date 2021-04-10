#include "driver.h"
#include "json.hpp"
#include <iostream>
#include <string>

// for convenience
using nlohmann::json;
using namespace std;

namespace
{
  // Checks if the SocketIO event has JSON data.
  // If there is data the JSON object in string format will be returned,
  // else the empty string "" will be returned.
  string hasData(string s)
  {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != string::npos)
    {
      return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
      return s.substr(b1, b2 - b1 + 1);
    }
    return "";
  }
}

Driver::Driver(int port)
: m_port{port}
{
}

void Driver::ConfigureSteeringPID(double Kp, double Ki, double Kd)
{
  m_pid_steering.Init(Kp, Ki, Kd);
}

void Driver::ConfigureThrottlePID(double Kp, double Ki, double Kd)
{
  m_pid_throttle.Init(Kp, Ki, Kd);
}

void Driver::SetMaxIterations(size_t max_iterations)
{
  m_max_iterations = max<size_t>(10, max_iterations);
}

double Driver::Run()
{
  // reset iteration counter and total squared error accumulator
  m_iteration = 0;
  m_acc_error = 0.0;

  ConfigureCallbacks();

  // start listening for WebSocket connections from the simulator on configured port
  if (!m_hub.listen(m_port))
  {
    cerr << "Failed to listen to port" << endl;
    return -1.0;
  }

  auto sparams = m_pid_steering.GetCoefficients();
  auto tparams = m_pid_throttle.GetCoefficients();
  cout << "Steering: Kp = " << sparams[0] << ", Ki = " << sparams[1] << ", Kd = " << sparams[2] << endl
       << "Throttle: Kp = " << tparams[0] << ", Ki = " << tparams[1] << ", Kd = " << tparams[2] << endl;

  // run the WebSocket server until the limit <max_iterations> of control messages has been reached
  m_hub.run();

  cout << "-> Accumulated error^2/speed: " << m_acc_error << endl;

  return m_acc_error;
}

double Driver::operator() (const vector<double>& params)
{
  // use function arguments as PID coefficients
  assert(params.size() == 6 && "Number of parameters must be 6.");
  m_pid_steering.Init(params[0], params[1], params[2]);
  m_pid_throttle.Init(params[3], params[4], params[5]);

  // force maximum number of controller iterations to be a finite number, so that ::Run
  // will eventually return
  if (m_max_iterations == 0)
  {
    m_max_iterations = 500;
  }

  return Run();
}

double Driver::GetSteeringValue() const
{
  constexpr double min_value = -1.0;
  constexpr double max_value = 1.0;

  double pid_error = m_pid_steering.TotalError();
//  return (max_value - min_value) * (erf(-pid_error) + 1.0) / 2.0 + min_value;
  double result = -pid_error;
  result = min(max_value, result);
  result = max(min_value, result);
  return result;
}

double Driver::GetThrottleValue(double steering_angle, double speed) const
{
//  return 0.45;
//
//  return (steering_angle > 10.0) ? 0.25 : 0.45;

  constexpr double min_value = 0.0;
  constexpr double max_value = 1.0;

  double pid_error = m_pid_throttle.TotalError();
//  double result = (max_value - min_value) * exp(-pid_error*pid_error) + min_value;

  double result = max_value - fabs(pid_error);
  result = min(max_value, result);
  result = max(min_value, result);

  /*if (steering_angle > 10.0)
  {
    result = 0.25;
  }
  else */
  if (speed < 15.0)
  {
    result = max(result, 1.0);
  }
//  else if (speed < 25.0 && steering_angle < 10.0)
//  {
//    result = max(result, 0.4);
//  }

  return result;
}

void Driver::ConfigureCallbacks()
{
  m_hub.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode /*opCode*/) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(string(data).substr(0, length));

      if (s != "")
      {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry")
        {
          m_iteration++;

          // on the first iteration, reset the simulator
          if (m_iteration == 1)
          {
            string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            return;
          }

          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());

          if (m_iteration < 5)
          {
            // ignore the first couple of iterations (some might be data from before the simulator reset)
            cte = speed = angle = 0.0;
          }

          // update controller errors
          m_pid_steering.UpdateError(cte);
          m_pid_throttle.UpdateError(cte);

          // update squared total error for optimization
          m_acc_error += (cte * cte) / max(1.0, speed);

          // Calculate steering value, range is [-1, 1]
          double steer_value = GetSteeringValue();

          // Calculate throttle value, range is [-1, 1]
          double throttle_value = GetThrottleValue(fabs(steer_value) * 25.0, speed);

          // DEBUG
//          cout << "CTE: " << cte << ", angle: " << angle << ", speed: " << speed << endl;
//          cout << "-> steering: " << steer_value << ", throttle: " << throttle_value << endl;

          // send steering angle and throttle to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          // when the maximum number of iterations is reached, close the connection
          if (m_iteration > m_max_iterations)
          {
            m_hub.uWS::Group<uWS::SERVER>::close();
            return;
          }
        }
      }
      else
      {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage
}
