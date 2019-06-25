#include "kalman_filter.h"

using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

/*
 * Please note that the Eigen library does not initialize
 *   Vector4d or Matrix4d objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() : noise_ax(9.0), noise_ay(9.0) {
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
  double dt2 = delta_t * delta_t;
  double dt3 = dt2 * delta_t / 2.0;
  double dt4 = dt2 * dt2 / 4.0;
  Q_ << noise_ax * dt4, 0, noise_ax * dt3, 0, 0, noise_ay * dt4, 0,
      noise_ay * dt3, noise_ax * dt3, 0, noise_ax * dt2, 0, 0, noise_ay * dt3,
      0, noise_ay * dt2;
};

void KalmanFilter::Predict() {
  /**
   * predict the state
   */
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter::Update(const LaserMeasurement m) {
  /**
   * update the state by using Kalman Filter equations
   */
  Vector2d y = m.raw_data_ - H_laser_ * x_;
  Matrix2d S = H_laser_ * P_ * H_laser_.transpose() + R_laser_;
  Eigen::Matrix<double, 4, 2> K = P_ * H_laser_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (Matrix4d::Identity() - K * H_laser_) * P_;
}

void KalmanFilter::Update(const RadarMeasurement m) {
  /**
   *update the state by using Extended Kalman Filter equations
   */
  H_radar_ = tools.CalculateJacobian(m.cart_);

  Vector3d h;
  h(0) = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  if (0.000001 > x_(0) * x_(0)) {
    x_(0) += 0.002;
  }
  h(1) = atan2(x_(1), x_(0));
  h(2) = (x_(0) * x_(2) + x_(1) * x_(3)) / h(0);
  Vector3d y = m.raw_data_ - h;
  y(1) = fmod(y(1), 2 * M_PI);
  (y(1) < M_PI ? true : y(1) = y(1) - 2 * M_PI);
  Matrix3d S = H_radar_ * P_ * H_radar_.transpose() + R_radar_;
  Eigen::Matrix<double, 4, 3> K = P_ * H_radar_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (Matrix4d::Identity() - K * H_radar_) * P_;
}
