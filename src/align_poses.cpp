#include <ze/geometry/align_poses.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>

namespace ze {

PoseAligner::PoseAligner(
    const TransformationVector& T_W_A,
    const TransformationVector& T_W_B)
  : T_W_A_(T_W_A)
  , T_W_B_(T_W_B)
{
  CHECK_EQ(T_W_A_.size(), T_W_B_.size());
}

double PoseAligner::evaluateError(
    const Transformation& T_A_B,
    HessianMatrix* H,
    GradientVector *g)
{
  double chi2 = 0.0;

  // Compute prediction error.
  Matrix6X residuals(6, T_W_A_.size());
  for(size_t i = 0; i < T_W_A_.size(); ++i)
  {
    residuals.col(i) =
        (Vector6() << T_W_A_[i] * T_A_B.getPosition() - T_W_B_[i].getPosition(),
                      Quaternion::log(T_W_B_[i].getRotation().inverse()
                                      * T_W_A_[i].getRotation()
                                      * T_A_B.getRotation())).finished();
  }
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
  residuals /= measurement_sigma_;

  if(H && g)
  {
    for(size_t i = 0; i < T_W_A_.size(); ++i)
    {
      // Compute Jacobian (if necessary, this can be optimized a lot).
      Matrix66 J = dRelpose_dTransformation(T_A_B, T_W_A_[i], T_W_B_[i]);

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

void PoseAligner::update(
    const Transformation& T_A_Bold,
    const UpdateVector& dx,
    Transformation& T_A_Bnew)
{
  T_A_Bnew = T_A_Bold * Transformation::exp(dx);
}

} // namespace ze


