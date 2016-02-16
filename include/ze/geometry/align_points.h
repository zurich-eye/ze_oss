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
      GradientVector *g);

  void update(
      const Transformation& T_A_Bold,
      const UpdateVector& dx,
      Transformation& T_A_Bnew);

private:
  FloatType measurement_sigma_;
  const Positions& p_A_;
  const Positions& p_B_;
};

/*!
 * @brief Jacobian of the norm of a point w.r.t. components of the point.
 *
 * f = norm(p) = sqrt(x^2 + y^2 + z^2), with p = [x, y, z].
 *
 * df/dp = 1 / sqrt(x^2 + y^2 + z^2) * [ x, y, z ]
 */
inline Matrix13 dNorm_dPoint(const Eigen::Ref<const Vector3>& p)
{
  return (1.0f / p.norm()) * p.transpose();
}

/*!
 * @brief Jacobian of the norm of a point w.r.t. components of the point.
 *
 * f = norm(p_A - T_A_B * p_B)
 *
 * Perturbation: R <- R * exp(dR), t <- t + R * dt
 * f = norm(p_A - (R * exp(dR) * p_B + t + R * dt)
 *   = norm(p_A - (R * (I + skew(dR)) * p_B + t + R * dt)
 *   = norm(p_A - (R * p_B - R * skew(p_B) * dR + t + R * dt)
 *
 * df/dR = dNorm_dPoint * R * skew(p_B)
 * df/dt = - dNorm_dPoint * R
 */
inline Matrix16 dNorm_dPose(const Transformation& T_A_B,
                            const Eigen::Ref<const Vector3>& p_A,
                            const Eigen::Ref<const Vector3>& p_B)
{
  Matrix3 R = T_A_B.getRotationMatrix();
  Matrix13 J_norm = dNorm_dPoint(p_A - T_A_B * p_B);
  Matrix16 J;
  J.head<3>() = - J_norm * R;
  J.tail<3>() = J_norm * R * skewSymmetric(p_B);
  return J;
}

} // namespace ze
