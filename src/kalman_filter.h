#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "measurement_package.h"
#include "tools.h"

class KalmanFilter {
 private:
  // measurement noise
  const double noise_ax;
  const double noise_ay;

  // state transition matrix
  Eigen::Matrix4d F_;

  // process covariance matrix
  Eigen::Matrix4d Q_;

  // measurement matrices
  LaserJacobian H_laser_;
  RadarJacobian H_radar_;

  // measurement covariance matrices
  Eigen::Matrix2d R_laser_;
  Eigen::Matrix3d R_radar_;

  // tool object used to compute Jacobian and RMSE
  Tools tools;

 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter() = default;

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   */
  void Init(const Eigen::Vector4d &x_in, const Eigen::Matrix4d &P_in);

  /**
   * Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   * @param delta_t time difference since last update
   */
  void UpdateProcess(const double &delta_t);
  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict();

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The laser measurement at k+1
   */
  void Update(const LaserMeasurement m);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The radar measurement at k+1
   */
  void Update(const RadarMeasurement m);

  // state vector
  Eigen::Vector4d x_;
  // state covariance matrix
  Eigen::Matrix4d P_;
};

#endif  // KALMAN_FILTER_H_
