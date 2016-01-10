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

  //! Landmark positions. Must have same length
  Positions p_W;

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;
};

//! Optimizes body pose by minimizing difference between bearing vectors.
class PoseOptimizer :
    public LeastSquaresSolver<6, Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<double>;
  using WeightFunction = TukeyWeightFunction<double>;

  PoseOptimizer(
      const std::vector<PoseOptimizerFrameData>& data,
      const Transformation& T_B_W_prior,
      const double prior_weight_pos, const double prior_weight_rot);

  double evaluateError(
      const Transformation& T_B_W, HessianMatrix* H, GradientVector *g);

  void update(
      const Transformation& T_Bold_W, const UpdateVector& dx,
      Transformation& T_Bnew_W);

private:
  const std::vector<PoseOptimizerFrameData>& data_;
  std::vector<double> measurement_sigma_;

  // Prior:
  const Transformation& T_B_W_prior_;
  double prior_weight_pos_;
  double prior_weight_rot_;
};

} // namespace ze
