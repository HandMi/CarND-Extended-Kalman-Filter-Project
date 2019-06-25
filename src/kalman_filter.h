#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include "measurement_package.h"
#include "tools.h"

class KalmanFilter {
 private:
  // tool object used to compute Jacobian and RMSE
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

  Tools tools;

 public:
  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   * @param P_in Initial state covariance
   * @param F_in Transition matrix
   * @param H_in Measurement matrix
   * @param R_in Measurement covariance matrix
   * @param Q_in Process covariance matrix
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
   * @param z The measurement at k+1
   */
  void Update(const LaserMeasurement m);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const RadarMeasurement m);

  // state vector
  Eigen::Vector4d x_;
  // state covariance matrix
  Eigen::Matrix4d P_;
};

#endif  // KALMAN_FILTER_H_
