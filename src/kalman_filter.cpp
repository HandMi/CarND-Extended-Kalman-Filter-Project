#include "kalman_filter.h"

using Eigen::Matrix2d;
using Eigen::Matrix4d;
using Eigen::Vector4d;

/*
 * Please note that the Eigen library does not initialize
 *   Vector4d or Matrix4d objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {
  // These should be defined as const but initialising const matrices in Eigen
  // is not pretty
  // measurement matrix - laser
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;
  // measurement covariance matrix - laser
  R_laser_ << 0.0225, 0, 0, 0.0225;
  // measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  // These matrices are time dependent
  // state transition matrix
  F_ = Matrix4d::Identity();
  // process covariance matrix
  Q_ = Matrix4d::Identity();
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(const Vector4d &x_in, const Matrix4d &P_in) {
  x_ = x_in;
  P_ = P_in;
}

void KalmanFilter::UpdateProcess(const double &delta_t) {
  // Update state transition matrix
  F_ = Matrix4d::Identity();
  F_.topRightCorner(2, 2) = Matrix2d::Identity() * delta_t;

  double noise_ax = 9.0;
  double noise_ay = 9.0;
};
void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
}

void KalmanFilter::Update(const Vector4d &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
}

void KalmanFilter::UpdateEKF(const Vector4d &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
}
