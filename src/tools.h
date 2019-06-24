#ifndef TOOLS_H_
#define TOOLS_H_

#include <vector>
#include "Eigen/Dense"

typedef Eigen::Matrix<double, 3, 4> RadarJacobian;
typedef Eigen::Matrix<double, 2, 4> LaserJacobian;

class Tools {
 public:
  /**
   * Constructor.
   */
  Tools();

  /**
   * Destructor.
   */
  virtual ~Tools();

  /**
   * A helper method to calculate RMSE.
   */
  Eigen::Vector4d CalculateRMSE(
      const std::vector<Eigen::Vector4d> &estimations,
      const std::vector<Eigen::Vector4d> &ground_truth);

  /**
   * A helper method to calculate Jacobians.
   */
  RadarJacobian CalculateJacobian(const Eigen::Vector4d &x_state);
};

#endif  // TOOLS_H_
