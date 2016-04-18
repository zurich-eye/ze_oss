#include <ze/geometry/align_points.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>

namespace ze {

PointAligner::PointAligner(
    const Positions& p_A,
    const Positions& p_B)
  : p_A_(p_A)
  , p_B_(p_B)
{
  CHECK_EQ(p_A_.cols(), p_B_.cols());
}

double PointAligner::evaluateError(
    const Transformation& T_A_B,
    HessianMatrix* H,
    GradientVector* g)
{
  double chi2 = 0.0;

  // Compute prediction error.
  Positions residuals = p_A_ - T_A_B.transformVectorized(p_B_);
  VectorX residuals_norm = residuals.colwise().norm();

  // At the first iteration, compute the scale of the error.
  if(iter_ == 0)
  {
    measurement_sigma_ = ScaleEstimator::compute(residuals_norm);
    VLOG(1) << "measurement sigma = " << measurement_sigma_;
  }

  // Robust cost function.
  VectorX weights =
      WeightFunction::weightVectorized(residuals_norm / measurement_sigma_);

  // Whiten error.
  residuals_norm /= measurement_sigma_;

  if(H && g)
  {
    for(int i = 0; i < p_A_.cols(); ++i)
    {
      // Compute Jacobian (if necessary, this can be optimized a lot).
      Matrix36 J = dPointdistance_dRelpose(T_A_B, p_A_.col(i), p_B_.col(i));

      // Whiten Jacobian.
      J /= measurement_sigma_;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * residuals.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  chi2 += 0.5 * weights.dot(residuals.colwise().squaredNorm());

  return chi2;
}

} // namespace ze


