#include "tools.h"
#include <iostream>

using Eigen::Array4d;
using Eigen::Matrix4d;
using Eigen::Vector4d;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

Vector4d Tools::CalculateRMSE(const vector<Vector4d> &estimations,
                              const vector<Vector4d> &ground_truth) {
  Array4d rmse(0, 0, 0, 0);
  if (estimations.size() > 0 && ground_truth.size() == estimations.size()) {
    for (uint i = 0; i < estimations.size(); i++) {
      rmse += pow(estimations[i].array() - ground_truth[i].array(), 2);
    }
    rmse /= estimations.size();
    return rmse.sqrt().matrix();
  } else {
    std::cout << "Error: Vector size mismatch in RMSE calculation."
              << std::endl;
    return rmse.matrix();
  }
}

RadarJacobian Tools::CalculateJacobian(const Vector4d &x_state) {
  RadarJacobian Hj;
  double px = x_state(0);
  double py = x_state(1);
  double p_square = px * px + py * py;
  double p_abs = sqrt(p_square);
  double vx = x_state(2);
  double vy = x_state(3);
  if (p_square > 0.0) {
    Hj << px / p_abs, py / p_abs, 0, 0, -py / p_square, px / p_square, 0, 0,
        py * (vx * py - vy * px) / (p_square * p_abs),
        px * (vy * px - vx * py) / (p_square * p_abs), px / p_abs, py / p_abs;
  } else {
    std::cout << "Error: Jacobian could not be computed" << std::endl;
    return Hj;
  }
}
