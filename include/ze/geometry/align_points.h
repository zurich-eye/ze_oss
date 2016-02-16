#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/least_squares_solver.h>

namespace ze {

//! Estimates relative transformation between two sets of associated pints
class PointAligner :
    public LeastSquaresSolver<6, Transformation, PointAligner>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<FloatType>;
  using WeightFunction = TukeyWeightFunction<FloatType>;

  PointAligner(
      const Positions& p_A,
      const Positions& p_B);

  double evaluateError(
      const Transformation& T_A_B,
      HessianMatrix* H,
      GradientVector* g);

  void update(
      const Transformation& T_A_Bold,
      const UpdateVector& dx,
      Transformation& T_A_Bnew);

private:
  FloatType measurement_sigma_;
  const Positions& p_A_;
  const Positions& p_B_;
};

inline Matrix36 dPointdistance_dRelpose(
    const Transformation& T_A_B,
    const Eigen::Ref<const Vector3>& p_A,
    const Eigen::Ref<const Vector3>& p_B)
{
  Matrix3 R = T_A_B.getRotationMatrix();
  Matrix36 J;
  J.block<3,3>(0,0) = - R; // translation
  J.block<3,3>(0,3) = R * skewSymmetric(p_B); // orientation
  return J;
}

} // namespace ze
