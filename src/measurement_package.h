#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
 public:
  MeasurementPackage(long long timestamp, Eigen::Vector4d data)
      : timestamp_(timestamp), cart_(data){};
  const long long timestamp_;
  const Eigen::Vector4d cart_;
};

// Laser Measurement is constructed from cartesian coordinates, no velocity data
class LaserMeasurement : public MeasurementPackage {
 public:
  LaserMeasurement(long long timestamp, double px, double py)
      : MeasurementPackage(timestamp, Eigen::Vector4d(px, py, 0, 0)),
        raw_data_(px, py){};
  const Eigen::Vector2d raw_data_;
};

// Radar Measurement is constructed from polar coordinates
class RadarMeasurement : public MeasurementPackage {
 public:
  RadarMeasurement(long long timestamp, double rho, double theta,
                   double rho_dot)
      : MeasurementPackage(
            timestamp,
            Eigen::Vector4d(rho * cos(theta), rho * sin(theta),
                            rho_dot * cos(theta), rho_dot * sin(theta))),
        raw_data_(rho, theta, rho_dot){};
  const Eigen::Vector3d raw_data_;
};

#endif  // MEASUREMENT_PACKAGE_H_
