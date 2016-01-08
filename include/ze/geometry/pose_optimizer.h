#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/least_squares_solver.h>

namespace ze {

class PoseOptimizer : public LeastSquaresSolver<6, Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using WeightFunction = TukeyWeightFunction<double>;

  PoseOptimizer(const Keypoints& measurements,
                const Positions& landmarks,
                const Transformation& T_C_B,
                const PinholeCamera& cam,
                double pixel_measurement_sigma);

  double evaluateError(const Transformation& T_B_W, HessianMatrix* H,
                       GradientVector *g);

  void update(const Transformation& T_Bold_W, const UpdateVector& dx,
              Transformation& T_Bnew_W);

private:
  const Keypoints& px_;  ///< Bearing vectors corresponding to feature measurements.
  const Positions& p_W_;    ///< 3D points corresponding to features.
  const Transformation& T_C_B_;   ///< Camera-IMU extrinsic calibration.
  const PinholeCamera& cam_;
  double px_measurement_sigma_ = 1.0;
};

} // namespace ze
