#include "runner.h"
#include "json.hpp"
#include <iostream>
#include <string>

// for convenience
using nlohmann::json;
using namespace std;

namespace
{
  double minMax(double value, double min, double max)
  {
    value = std::min(value, max);
    value = std::max(value, min);
    return value;
  }

  double errorFunction(double value, double min, double max)
  {
      return (max - min) * (erf(value) + 1.0) / 2.0 + min;
  }

  double gaussianFunction(double value, double min, double max)
  {
      return (max - min) * exp(-value*value / 2.0) + min;
  }

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

void Runner::ConfigureSteeringPID(double Kp, double Ki, double Kd)
{
  m_pid_steering.Init(Kp, Ki, Kd);
}

void Runner::ConfigureThrottlePID(double Kp, double Ki, double Kd)
{
  m_pid_throttle.Init(Kp, Ki, Kd);
}

void Runner::SetMaxIterations(size_t max_iterations)
{
  m_max_iterations = max_iterations;
}

double Runner::Run()
{
  // reset iteration counter and total squared error accumulator
  m_iteration = 0;
  m_total_sq_error = 0.0;

  ConfigureCallbacks();

  // start listening for WebSocket connections from the simulator on configured port
  if (m_hub.listen(m_port))
  {
//    cout << "Listening to port " << m_port << endl;
  }
  else
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

  cout << "-> Accumulated error^2: " << m_total_sq_error << endl;

  return m_total_sq_error;
}

void Runner::SetOptimizingMode(Mode mode)
{
  m_mode = mode;
}

double Runner::operator() (const vector<double>& params)
{
  // use function arguments as PID coefficients, depending on configured optimization mode
  if (m_mode == Mode::Both)
  {
    assert(params.size() == 6 && "Number of parameters must be 6 when optimizing both steering and throttle.");
    m_pid_steering.Init(params[0], params[1], params[2]);
    m_pid_throttle.Init(params[3], params[4], params[5]);
  }
  else if (m_mode == Mode::Steering)
  {
    assert(params.size() == 3 && "Number of parameters must be 3 when optimizing steering only.");
    m_pid_steering.Init(params[0], params[1], params[2]);
  }
  else if (m_mode == Mode::Throttle)
  {
    assert(params.size() == 3 && "Number of parameters must be 3 when optimizing throttle only.");
    m_pid_throttle.Init(params[0], params[1], params[2]);
  }

  // force maximum number of controller iterations to be a finite number
  if (m_max_iterations == 0)
  {
    m_max_iterations = 500;
  }

  return Run();
}

void Runner::ConfigureCallbacks()
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
          else if (m_iteration == 2)
          {
            // discard first input of a cycle
            return;
          }

          // j[1] is the data JSON object
          double cte = stod(j[1]["cte"].get<string>());
          double speed = stod(j[1]["speed"].get<string>());
          double angle = stod(j[1]["steering_angle"].get<string>());

          m_pid_steering.UpdateError(cte);
          m_pid_throttle.UpdateError(cte);

          // Update squared total error for optimization
          double steering_error = m_pid_steering.TotalError();
          double throttle_error = m_pid_throttle.TotalError();
          m_total_sq_error += steering_error * steering_error + throttle_error * throttle_error;

          // Calculate steering value, range is [-1, 1]
          double steer_value = errorFunction(-steering_error, -1.0, 1.0);

          // Calculate throttle value, range is [-1, 1]
//          double throttle_value = 0.4;
          double throttle_value = gaussianFunction(throttle_error, -0.25, 1.0);
          if (speed < 10.0)
          {
            throttle_value = max(throttle_value, 0.2);
          }

           /**
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */

          // DEBUG
//          cout << "CTE: " << cte << ", angle: " << angle << ", speed: " << speed << endl;
//          cout << "-> steering: " << steer_value << ", throttle: " << throttle_value << endl;

          // When the maximum number of iterations is reached, close the connection.
          if (m_iteration > m_max_iterations)
          {
            m_hub.uWS::Group<uWS::SERVER>::close();
            return;
          }

          // send steering angle and throttle to the simulator
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

//    m_hub.onConnection([](uWS::WebSocket<uWS::SERVER> /*ws*/, uWS::HttpRequest /*req*/) {
//      cout << "Connected!!!" << endl;
//    });

//    m_hub.onDisconnection([](uWS::WebSocket<uWS::SERVER> /*ws*/, int /*code*/, char* /*message*/, size_t /*length*/) {
//      cout << "Disconnected." << endl;
//    });
}
