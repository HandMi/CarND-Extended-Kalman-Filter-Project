#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  enum SensorType { LASER, RADAR };
  MeasurementPackage(SensorType sensor_type, long long timestamp,
                     Eigen::Vector4d data)
      : sensor_type_(sensor_type), timestamp_(timestamp), cart_(data){};

  const SensorType sensor_type_;
  const long long timestamp_;
  const Eigen::Vector4d cart_;
};

class LaserMeasurement : public MeasurementPackage {
 public:
  LaserMeasurement(long long timestamp, float px, float py)
      : MeasurementPackage(LASER, timestamp, Eigen::Vector4d(px, py, 0, 0)){};
};

class RadarMeasurement : public MeasurementPackage {
 public:
  RadarMeasurement(long long timestamp, float rho, float theta, float rho_dot)
      : MeasurementPackage(
            RADAR, timestamp,
            Eigen::Vector4d(rho * cos(theta), rho * sin(theta),
                            rho_dot * cos(theta), rho_dot * sin(theta))){};
};

#endif  // MEASUREMENT_PACKAGE_H_
