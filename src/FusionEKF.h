#ifndef FusionEKF_H_
#define FusionEKF_H_

#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "kalman_filter.h"
#include "measurement_package.h"
#include "tools.h"

class FusionEKF {
 public:
  /**
   * Destructor.
   */
  virtual ~FusionEKF() = default;

  /**
   * Run the whole flow of the Kalman Filter from here.
   */
  template <typename MeasurementPackageType>
  void ProcessMeasurement(const MeasurementPackageType &measurement_pack);

  /**
   * Kalman Filter update and prediction math lives in here.
   */
  KalmanFilter ekf_;

 private:
  // check whether the tracking toolbox was initialized or not (first
  // measurement)
  bool is_initialized_ = false;

  // previous timestamp
  long long previous_timestamp_ = 0;
};

template <typename MeasurementPackageType>
void FusionEKF::ProcessMeasurement(
    const MeasurementPackageType &measurement_pack) {
  /**
   * Initialization
   */
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    std::cout << "EKF: " << std::endl;
    ekf_.Init(measurement_pack.cart_, Eigen::Matrix4d::Identity());
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  double delta_t =
      static_cast<double>(measurement_pack.timestamp_ - previous_timestamp_) /
      1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.UpdateProcess(delta_t);
  ekf_.Predict();

  /**
   * Update
   */

  ekf_.Update(measurement_pack);

  // print the outputstd::
  std::cout << "x_ = " << std::endl << ekf_.x_ << std::endl;
  std::cout << "P_ = " << std::endl << ekf_.P_ << std::endl;
}

#endif  // FusionEKF_H_
