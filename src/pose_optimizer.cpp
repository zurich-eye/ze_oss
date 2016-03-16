#include <ze/geometry/pose_optimizer.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>
#include <ze/geometry/pose_prior.h>

namespace ze {

PoseOptimizer::PoseOptimizer(
    const std::vector<PoseOptimizerFrameData>& data,
    const Transformation& T_B_W_prior,
    const FloatType prior_weight_pos, const FloatType prior_weight_rot)
  : data_(data)
  , T_B_W_prior_(T_B_W_prior)
  , prior_weight_pos_(prior_weight_pos)
  , prior_weight_rot_(prior_weight_rot)
{
  // The scale of the error is computed at the first iteration.
  measurement_sigma_.resize(data.size());
}

FloatType PoseOptimizer::evaluateError(
    const Transformation& T_B_W, HessianMatrix* H, GradientVector* g)
{
  CHECK_EQ(data_.size(), measurement_sigma_.size());
  FloatType chi2 = 0.0;

  // Loop over all cameras in rig.
  for (size_t i = 0; i < data_.size(); ++i)
  {
    const PoseOptimizerFrameData& data = data_[i];
    FloatType& measurement_sigma = measurement_sigma_[i];

    if(data.f.cols() == 0)
    {
      LOG(WARNING) << "Have no measurements in camera " << i;
      continue;
    }

    // Transform points from world coordinates to camera coordinates.
    const Transformation T_C_W = data.T_C_B * T_B_W;
    Positions p_C = T_C_W.transformVectorized(data.p_W);

    // Normalize points to obtain estimated bearing vectors.
    Bearings f_est = p_C;
    normalizeBearings(f_est);

    // Compute difference between bearing vectors.
    Bearings f_err = f_est - data.f;
    VectorX f_err_norm = f_err.colwise().norm();

    // At the first iteration, compute the scale of the error.
    if (iter_ == 0)
    {
      measurement_sigma = ScaleEstimator::compute(f_err_norm);
      VLOG(300) << "Cam " << i << " measurement sigma = " << measurement_sigma;
    }

    // Robust cost function.
    VectorX weights =
        WeightFunction::weightVectorized(f_err_norm / measurement_sigma);

    // Whiten error.
    f_err /= measurement_sigma;

    if (H && g)
    {
      const Matrix3 R_C_W = T_C_W.getRotationMatrix();
      const int n = data.f.cols();
      Matrix36 G;
      G.block<3,3>(0,0) = I_3x3;
      for(int i = 0; i < n; ++i)
      {
        // Jacobian computation.
        G.block<3,3>(0,3) = - skewSymmetric(data.p_W.col(i));
        Matrix3 J_normalization = dBearing_dLandmark(p_C.col(i));
        Matrix36 J = J_normalization * R_C_W * G;

        // Whiten Jacobian.
        J /= measurement_sigma;

        // Compute Hessian and Gradient Vector.
        H->noalias() += J.transpose() * J * weights(i);
        g->noalias() -= J.transpose() * f_err.col(i) * weights(i);
      }
    }

    // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
    chi2 += 0.5 * weights.dot(f_err.colwise().squaredNorm());
  }

  // Apply prior.
  if (prior_weight_rot_ > 0.0f || prior_weight_pos_ > 0.0f)
  {
    applyPosePrior(T_B_W, T_B_W_prior_, prior_weight_rot_, prior_weight_pos_, *H, *g);
  }

  return chi2;
}

} // namespace ze


