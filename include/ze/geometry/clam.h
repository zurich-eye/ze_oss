#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/lsq_solver.h>
#include <ze/geometry/lsq_state.h>

namespace ze {

using ClamState = State<Transformation, VectorX>;

//! Data required by Clam per frame.
struct ClamFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Measurements: Bearing vectors.
  Bearings f;

  //! Landmark positions expressed in body of reference frame.
  //! Each column corresponds to a bearing measurement.
  Positions p_Br;

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;
};

struct FeatureTrack
{
  uint32_t ref_index_;

};

//! Coupled localization and mapping (Clam).
class Clam : public LeastSquaresSolver<ClamState, Clam>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<FloatType>;
  using WeightFunction = TukeyWeightFunction<FloatType>;

  Clam(
      const std::vector<ClamFrameData>& data,
      const Transformation& T_B_W_prior,
      const FloatType prior_weight_pos,
      const FloatType prior_weight_rot);

  FloatType evaluateError(
      const Transformation& T_B_W,
      HessianMatrix* H,
      GradientVector *g);

private:
  const std::vector<ClamFrameData>& data_;
  std::vector<FloatType> measurement_sigma_;

  // Prior:
  const Transformation& T_B_W_prior_;
  FloatType prior_weight_pos_;
  FloatType prior_weight_rot_;
};

} // namespace ze
