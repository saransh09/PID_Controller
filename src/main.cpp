#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <cstdlib>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char ** argv)
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  double kp,ki,kd;
  char *pEnd;
  bool tune;
  if(argc==5){
    kp = std::strtod(argv[1],&pEnd);
    ki = std::strtod(argv[2],&pEnd);
    kd = std::strtod(argv[3],&pEnd);
    if(std::atoi(argv[4])){
      tune = true;
      std::cout<<" Twiddle Mode!" << std::endl;
    }
    else{
      tune = false;
    }
  }
  else if(argc == 1){
    kp = 1.30; // manually setting the parameters
    ki = 0.01; // manually setting the parameters
    kd = 6.02; // manually setting the parameters
    tune = false;
  }
  else{
    std::cout<<"Arguments missing, please enter the Kp,Ki and Kd initial params" << std::endl;
    std::cout<<"./pid followed by Kp,Ki,Kd and TwiddleToggle(0/1) during the run command" << std::endl;
    return -1;
  }
  std::cout << kp << "," << ki << "," << kd << std::endl;
  pid.Init(kp,ki,kd,tune);


  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();

          // check for the overshooting of the steer value
          if (steer_value < -1){
            steer_value = -0.7;
          }
          if (steer_value > 1){
            steer_value = 0.7;
          }

          // throttle conditions to maintain good speed for the car
          double throttle;
          if (fabs(angle) < 2.5 && speed < 25){
            throttle = 0.8;
          }
          else if(fabs(angle) < 7.5){
            throttle = 0.3;
          }
          else{
            throttle = 0.1;
          }
          
          // DEBUG and twiddle mode
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          if(pid.isTwiddle && (pid.step > pid.max_step*50 || pid.err > pid.best_error*2)){  
            std::vector<double> gain = pid.doTwiddle();
            pid.Init(gain[0],gain[1],gain[2],true);
            std::string reset_msg = "42[\"reset\",{}]";
            std::cout << "Restart!" << std::endl;
            ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
          }
          std::cout << "Step: "<< pid.step << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          std::cout << "Kp: "<< pid.Kp << " Ki: " << pid.Ki << " Kd: " << pid.Kd << std::endl;
          std::cout << "d_Kp: "<< pid.dp[0] << " d_Ki: " << pid.dp[1] << " Kd: " << pid.dp[2] << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        pid.step += 1;
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
