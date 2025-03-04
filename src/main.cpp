#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include "FusionEKF.h"
#include "json.hpp"
#include "tools.h"

using Eigen::Matrix4d;
using Eigen::Vector4d;
using std::string;
using std::vector;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<Vector4d> estimations;
  vector<Vector4d> ground_truth;

  h.onMessage([&fusionEKF, &tools, &estimations, &ground_truth](
                  uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                  uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          string sensor_measurement = j[1]["sensor_measurement"];

          std::istringstream iss(sensor_measurement);

          long long timestamp;

          // reads first element from the current line
          string sensor_type;
          iss >> sensor_type;

          if (sensor_type.compare("L") == 0) {
            float px;
            float py;
            iss >> px;
            iss >> py;
            iss >> timestamp;
            LaserMeasurement meas_package(timestamp, px, py);
            // Call ProcessMeasurement(meas_package) for Kalman filter
            fusionEKF.ProcessMeasurement(meas_package);
          } else if (sensor_type.compare("R") == 0) {
            float rho;
            float theta;
            float rho_dot;
            iss >> rho;
            iss >> theta;
            iss >> rho_dot;
            iss >> timestamp;
            RadarMeasurement meas_package(timestamp, rho, theta, rho_dot);
            // Call ProcessMeasurement(meas_package) for Extended Kalman filter
            fusionEKF.ProcessMeasurement(meas_package);
          }

          float x_gt;
          float y_gt;
          float vx_gt;
          float vy_gt;
          iss >> x_gt;
          iss >> y_gt;
          iss >> vx_gt;
          iss >> vy_gt;

          Vector4d gt_values(x_gt, y_gt, vx_gt, vy_gt);
          ground_truth.push_back(gt_values);

          // Push the current estimated x,y positon from the Kalman filter's
          // state vector

          estimations.push_back(fusionEKF.ekf_.x_);

          Vector4d RMSE = tools.CalculateRMSE(estimations, ground_truth);

          json msgJson;
          msgJson["estimate_x"] = fusionEKF.ekf_.x_(0);
          msgJson["estimate_y"] = fusionEKF.ekf_.x_(1);
          msgJson["rmse_x"] = RMSE(0);
          msgJson["rmse_y"] = RMSE(1);
          msgJson["rmse_vx"] = RMSE(2);
          msgJson["rmse_vy"] = RMSE(3);
          auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }  // end "telemetry" if

      } else {
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  });  // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }

  h.run();
}