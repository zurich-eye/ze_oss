#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/least_squares_solver.h>

namespace ze {

//! Data required by the pose-optimizer per frame.
struct PoseOptimizerFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Measurements: Bearing vectors.
  Bearings f;

  //! Landmark positions. Each column corresponds to a bearing measurement.
  Positions p_W;

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;
};

//! Optimizes body pose by minimizing difference between bearing vectors.
class PoseOptimizer :
    public LeastSquaresSolver<Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<FloatType>;
  using WeightFunction = TukeyWeightFunction<FloatType>;

  PoseOptimizer(
      const std::vector<PoseOptimizerFrameData>& data,
      const Transformation& T_B_W_prior,
      const FloatType prior_weight_pos, const FloatType prior_weight_rot);

  FloatType evaluateError(
      const Transformation& T_B_W, HessianMatrix* H, GradientVector *g);

private:
  const std::vector<PoseOptimizerFrameData>& data_;
  std::vector<FloatType> measurement_sigma_;

  // Prior:
  const Transformation& T_B_W_prior_;
  FloatType prior_weight_pos_;
  FloatType prior_weight_rot_;
};

/*!
 * @brief Jacobian of bearing vector w.r.t. landmark in camera coordinates.
 *
 * f = p / norm(p) = p / sqrt(x^2 + y^2 + z^2), with p = [x, y, z].
 *
 *                                       | y^2 + z^2, -xy, -xz |
 * df/dp = 1 / (x^2 + y^2 + z^2)^(3/2) * | -xy, x^2 + z^2, -yz |
 *                                       | -xz, -yz, x^2 + z^2 |.
 */
inline Matrix3 dBearing_dLandmark(const Position& p_C)
{
  const FloatType x2 = p_C(0) * p_C(0);
  const FloatType y2 = p_C(1) * p_C(1);
  const FloatType z2 = p_C(2) * p_C(2);
  const FloatType xy = p_C(0) * p_C(1);
  const FloatType xz = p_C(0) * p_C(2);
  const FloatType yz = p_C(1) * p_C(2);
  const FloatType x2_y2_z2 = x2 + y2 + z2;
  Matrix3 J;
  J << y2 + z2, -xy, -xz,
       -xy, x2 + z2, -yz,
       -xz, -yz, x2 + y2;
  J /= (x2_y2_z2 * std::sqrt(x2_y2_z2));
  return J;
}

} // namespace ze
