#include <ze/geometry/pose_optimizer.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>
#include <ze/geometry/pose_prior.h>

namespace ze {

PoseOptimizer::PoseOptimizer(
    std::vector<PoseOptimizerFrameData>& data,
    const Transformation& T_B_W_prior,
    const FloatType prior_weight_pos, const FloatType prior_weight_rot)
  : data_(data)
  , T_B_W_prior_(T_B_W_prior)
  , prior_weight_pos_(prior_weight_pos)
  , prior_weight_rot_(prior_weight_rot)
{}

FloatType PoseOptimizer::evaluateError(
    const Transformation& T_B_W, HessianMatrix* H, GradientVector* g)
{
  FloatType chi2 = 0.0;

  // Loop over all cameras in rig.
  for (auto& residual_block : data_)
  {
    switch (residual_block.type)
    {
      case PoseOptimizerResidualType::Bearing:
        chi2 += evaluateBearingErrors(T_B_W, iter_ == 0, residual_block, H, g);
        break;
      case PoseOptimizerResidualType::UnitPlane:
        chi2 += evaluateUnitPlaneErrors(T_B_W, iter_ == 0, residual_block, H, g);
        break;
      default:
        LOG(FATAL) << "Residual type not implemented.";
        break;
    }
  }

  // Apply prior.
  if (prior_weight_rot_ > 0.0f || prior_weight_pos_ > 0.0f)
  {
    applyPosePrior(T_B_W, T_B_W_prior_, prior_weight_rot_, prior_weight_pos_, *H, *g);
  }

  return chi2;
}

//------------------------------------------------------------------------------
FloatType evaluateBearingErrors(
    const Transformation& T_B_W,
    const bool first_iteration,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g)
{
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
  if (first_iteration)
  {
    data.measurement_sigma = PoseOptimizer::ScaleEstimator::compute(f_err_norm);
  }

  // Robust cost function.
  VectorX weights =
      PoseOptimizer::WeightFunction::weightVectorized(f_err_norm / data.measurement_sigma);

  // Whiten error.
  f_err /= data.measurement_sigma;

  if (H && g)
  {
    const Matrix3 R_C_W = T_C_W.getRotationMatrix();
    const int n = data.f.cols();
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3;
    for(int i = 0; i < n; ++i)
    {
      // Jacobian computation.
      G.block<3,3>(0,3) = -skewSymmetric(data.p_W.col(i));
      Matrix3 J_normalization = dBearing_dLandmark(p_C.col(i));
      Matrix36 J = J_normalization * R_C_W * G;

      // Whiten Jacobian.
      J /= data.measurement_sigma;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * f_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return 0.5 * weights.dot(f_err.colwise().squaredNorm());
}

//------------------------------------------------------------------------------
FloatType evaluateUnitPlaneErrors(
    const Transformation& T_B_W,
    const bool first_iteration,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g)
{
  if (first_iteration)
  {
    data.uv = project2Vectorized(data.f);
  }

  // Transform points from world coordinates to camera coordinates.
  const Transformation T_C_W = data.T_C_B * T_B_W;
  Positions p_C = T_C_W.transformVectorized(data.p_W);

  // Compute projection of points on unit-plane:
  Keypoints uv_est = project2Vectorized(p_C);

  // Compute difference between bearing vectors.
  Keypoints uv_err = uv_est - data.uv;
  VectorX uv_err_norm = uv_err.colwise().norm();

  // At the first iteration, compute the scale of the error.
  if (first_iteration)
  {
    data.measurement_sigma = PoseOptimizer::ScaleEstimator::compute(uv_err_norm);
  }

  // Robust cost function.
  VectorX weights =
      PoseOptimizer::WeightFunction::weightVectorized(uv_err_norm / data.measurement_sigma);

  // Whiten error.
  uv_err /= data.measurement_sigma;

  if (H && g)
  {
    const Matrix3 R_C_W = T_C_W.getRotationMatrix();
    const int n = data.f.cols();
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3;
    for(int i = 0; i < n; ++i)
    {
      // Jacobian computation.
      G.block<3,3>(0,3) = -skewSymmetric(data.p_W.col(i));
      Matrix23 J_proj = dUv_dLandmark(p_C.col(i));
      Matrix26 J = J_proj * R_C_W * G;

      // Whiten Jacobian.
      J /= data.measurement_sigma;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * uv_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return 0.5 * weights.dot(uv_err.colwise().squaredNorm());
}

} // namespace ze


