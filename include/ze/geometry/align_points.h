#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/lsq_solver.h>

namespace ze {

//! Estimates relative transformation between two sets of associated pints
class PointAligner : public LeastSquaresSolver<Transformation, PointAligner>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = UnitScaleEstimator<real_t>;
  using WeightFunction = UnitWeightFunction<real_t>;

  PointAligner(
      const Positions& p_A,
      const Positions& p_B);

  double evaluateError(
      const Transformation& T_A_B,
      HessianMatrix* H,
      GradientVector* g);

  //! Compute LSQ alignment in SE3 (closed form solution by K. S. Arun et al.:
  // Least-Squares Fitting of Two 3-D Point Sets, IEEE Trans. Pattern Anal.
  // Mach. Intell., 9, NO. 5, SEPTEMBER 1987)
  //! @param pts_A A vector of N points in the 'A' reference system (3xN)
  //! @param pts_B A vector of N points in the 'B' reference system (3xN)
  //! @return T_B_A such that ||T_B_A * pts_A - pts_B|| is minimized
  static Transformation alignSE3(
      const Positions& pts_A, const Positions& pts_B);

  //! Compute LSQ alignment in Sim3 (close form solution by S. Umeyama:
  //! Least-Squares Estimation of Transformation Parameters Between Two Point
  //! Patterns, IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.)
  //! @param pts_A A vector of N points in the 'A' reference system (3xN)
  //! @param pts_B A vector of N points in the 'B' reference system (3xN)
  //! @return <s, T_B_A> such that ||s*T_B_A * pts_A - pts_B|| is minimized
  static std::pair<real_t, Transformation> alignSim3(
      const Positions& pts_A, const Positions& pts_B);

private:
  real_t measurement_sigma_;
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
