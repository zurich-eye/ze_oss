#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/least_squares_solver.h>

namespace ze {

class PoseOptimizerBearingVectors :
    public LeastSquaresSolver<6, Transformation, PoseOptimizerBearingVectors>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using WeightFunction = UnitWeightFunction<double>;

  PoseOptimizerBearingVectors(
      const Bearings& measurements, const Positions& landmarks,
      const Transformation& T_C_B, double pixel_measurement_sigma);

  double evaluateError(
      const Transformation& T_B_W, HessianMatrix* H, GradientVector *g);

  void update(
      const Transformation& T_Bold_W, const UpdateVector& dx,
      Transformation& T_Bnew_W);

private:
  const Bearings& f_;  ///< Bearing vectors corresponding to feature measurements.
  const Positions& p_W_;    ///< 3D points corresponding to features.
  const Transformation& T_C_B_;   ///< Camera-IMU extrinsic calibration.
  double measurement_sigma_ = 1.0;
};

} // namespace ze
