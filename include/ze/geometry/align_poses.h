#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/lsq_solver.h>

namespace ze {

//! Estimates relative transformation between two sets of associated pints
class PoseAligner :
    public LeastSquaresSolver<Transformation, PoseAligner>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = UnitScaleEstimator<real_t>;
  using WeightFunction = UnitWeightFunction<real_t>;

  PoseAligner(
      const TransformationVector& T_W_A,
      const TransformationVector& T_W_B,
      const real_t measurement_sigma_pos,
      const real_t measurement_sigma_rot);

  double evaluateError(
      const Transformation& T_A_B,
      HessianMatrix* H,
      GradientVector *g);

private:
  const TransformationVector& T_W_A_;
  const TransformationVector& T_W_B_;
  real_t measurement_sigma_pos_;
  real_t measurement_sigma_rot_;
};

inline Matrix6 dRelpose_dTransformation(
    const Transformation& T_A0_B0,
    const Transformation& T_Ai_A0,
    const Transformation& T_B0_Bi)
{
  Quaternion R_error = T_Ai_A0.getRotation() * T_A0_B0.getRotation() * T_B0_Bi.getRotation();
  Matrix3 R_Ai_B0 = T_Ai_A0.getRotationMatrix() * T_A0_B0.getRotationMatrix();
  Matrix3 R_Bi_B0 = T_B0_Bi.getRotation().inverse().getRotationMatrix();
  Matrix6 J = Z_6x6;
  J.block<3,3>(0,0) = R_Ai_B0; // drt / dt
  J.block<3,3>(0,3) = - R_Ai_B0 * skewSymmetric(T_B0_Bi.getPosition()); // drt / dR
  J.block<3,3>(3,3) = logmapDerivativeSO3(R_error.log()) * R_Bi_B0; // drR / dR
  return J;
}

} // namespace ze
