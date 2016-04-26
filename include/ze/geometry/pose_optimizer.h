#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/lsq_solver.h>

namespace ze {

enum class PoseOptimizerResidualType
{
  Bearing,
  UnitPlane,
  UnitPlaneEdgelet
};

//! Data required by the pose-optimizer per frame.
struct PoseOptimizerFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PoseOptimizerResidualType type;

  //! Measurements: Bearing vectors corresponding to keypoints.
  Bearings f;

  //! Gradients: Used only for edgelets.
  Gradients grad;

  //! Measurements bookkeeping: Corresponding indices. (Not used by the actual algorithm).
  KeypointIndices kp_idx;

  //! Landmark positions. Each column corresponds to a bearing measurement.
  //! @todo(cfo): Use inverse depth parametrization or homogeneous points.
  Positions p_W;

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;

  //! @name Internal buffer
  //! @{
  //! Measurements: Projected on unit-plane. (computed internally).
  Keypoints uv;

  FloatType measurement_sigma;
  //! @}
};
using PoseOptimizerFrameDataVec = std::vector<PoseOptimizerFrameData>;

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
      std::vector<PoseOptimizerFrameData>& data,
      const Transformation& T_B_W_prior,
      const FloatType prior_weight_pos, const FloatType prior_weight_rot);

  FloatType evaluateError(
      const Transformation& T_B_W,
      HessianMatrix* H,
      GradientVector* g);

private:
  std::vector<PoseOptimizerFrameData>& data_;

  // Prior:
  const Transformation& T_B_W_prior_;
  FloatType prior_weight_pos_;
  FloatType prior_weight_rot_;
};

FloatType evaluateBearingErrors(
    const Transformation& T_B_W,
    const bool compute_measurement_sigma,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);

FloatType evaluateUnitPlaneErrors(
    const Transformation& T_B_W,
    const bool compute_measurement_sigma,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g);

/*!
 * @brief Jacobian of bearing vector w.r.t. landmark in camera coordinates.
 *
 * f = p / norm(p) = p / sqrt(x^2 + y^2 + z^2), with p = [x, y, z].
 *
 *                                       | y^2 + z^2, -xy, -xz |
 * df/dp = 1 / (x^2 + y^2 + z^2)^(3/2) * | -xy, x^2 + z^2, -yz |
 *                                       | -xz, -yz, x^2 + z^2 |.
 */
inline Matrix3 dBearing_dLandmark(const Eigen::Ref<const Position>& p_C)
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

/*!
 * @brief Jacobian of unit-plane coordinates uv w.r.t. landmark in camera coordinates.
 */
inline Matrix23 dUv_dLandmark(const Eigen::Ref<const Position>& p_C)
{
  const FloatType z_sq = p_C(2) * p_C(2);
  const FloatType z_inv = 1.0 / p_C(2);
  Matrix23 J;
  J << z_inv, 0.0, -p_C(0) / z_sq,
       0.0, z_inv, -p_C(1) / z_sq;
  return J;
}

} // namespace ze
