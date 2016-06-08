#include <ze/geometry/pose_optimizer.h>

#include <cmath>
#include <numeric>
#include <algorithm>

#include <ze/cameras/camera.h>
#include <ze/common/logging.hpp>
#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>
#include <ze/geometry/pose_prior.h>

namespace ze {

//------------------------------------------------------------------------------
PoseOptimizer::PoseOptimizer(
    const LeastSquaresSolverOptions& options,
    std::vector<PoseOptimizerFrameData>& data)
  : LeastSquaresSolver<Transformation, PoseOptimizer>(options)
  , data_(data)
{}

//------------------------------------------------------------------------------
PoseOptimizer::PoseOptimizer(
    const LeastSquaresSolverOptions& options,
    std::vector<PoseOptimizerFrameData>& data,
    const Transformation& T_B_W_prior,
    const FloatType prior_weight_pos,
    const FloatType prior_weight_rot)
  : LeastSquaresSolver<Transformation, PoseOptimizer>(options)
  , data_(data)
  , T_B_W_prior_(T_B_W_prior)
  , prior_weight_pos_(prior_weight_pos)
  , prior_weight_rot_(prior_weight_rot)
{}

//------------------------------------------------------------------------------
LeastSquaresSolverOptions PoseOptimizer::getDefaultSolverOptions()
{
  LeastSquaresSolverOptions options;
  options.strategy = SolverStrategy::GaussNewton;
  options.max_iter = 10;
  options.eps = 0.000001;
  return options;
}

//------------------------------------------------------------------------------
void PoseOptimizer::setPrior(
    const Transformation& T_B_W_prior,
    const FloatType prior_weight_pos,
    const FloatType prior_weight_rot)
{
  T_B_W_prior_ = T_B_W_prior;
  prior_weight_pos_ = prior_weight_pos;
  prior_weight_rot_ = prior_weight_rot;
}

//------------------------------------------------------------------------------
FloatType PoseOptimizer::evaluateError(
    const Transformation& T_B_W, HessianMatrix* H, GradientVector* g)
{
  FloatType chi2 = FloatType{0.0};

  // Loop over all cameras in rig.
  VLOG(400) << "Num residual blocks = " << data_.size();
  for (auto& residual_block : data_)
  {
    VLOG(400) << "Process residual block " << residual_block.camera_idx;
    switch (residual_block.type)
    {
      case PoseOptimizerResidualType::Bearing:
        chi2 += evaluateBearingErrors(T_B_W, iter_ == 0, residual_block, H, g).first;
        break;
      case PoseOptimizerResidualType::UnitPlane:
        chi2 += evaluateUnitPlaneErrors(T_B_W, iter_ == 0, residual_block, H, g).first;
        break;
      case PoseOptimizerResidualType::Line:
        chi2 += evaluateLineErrors(T_B_W, iter_ == 0, residual_block, H, g).first;
        break;
      default:
        LOG(FATAL) << "Residual type not implemented.";
        break;
    }
  }

  // Apply prior.
  if (prior_weight_rot_ > FloatType{0.0} || prior_weight_pos_ > FloatType{0.0})
  {
    applyPosePrior(T_B_W, T_B_W_prior_, prior_weight_rot_, prior_weight_pos_, *H, *g);
  }

  return chi2;
}

//------------------------------------------------------------------------------
std::pair<FloatType, VectorX> evaluateBearingErrors(
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

  // Account that features at higher levels have higher uncertainty.
  f_err_norm.array() /= data.scale.array();

  // At the first iteration, compute the scale of the error.
  if (first_iteration)
  {
    data.measurement_sigma = PoseOptimizer::ScaleEstimator::compute(f_err_norm);
  }

  // Robust cost function.
  VectorX weights = PoseOptimizer::WeightFunction::weightVectorized(
                      f_err_norm.array() / data.measurement_sigma);

  // Instead of whitening the error and the Jacobian, we apply sigma to the weights:
  weights.array() /= (data.scale.array() * data.measurement_sigma * data.measurement_sigma);

  if (H && g)
  {
    const Matrix3 R_C_W = T_C_W.getRotationMatrix();
    const int n = data.f.cols();
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3;
    for (int i = 0; i < n; ++i)
    {
      // Jacobian computation.
      G.block<3,3>(0,3) = -skewSymmetric(data.p_W.col(i));
      Matrix3 J_normalization = dBearing_dLandmark(p_C.col(i));
      Matrix36 J = J_normalization * R_C_W * G;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * f_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return std::make_pair(FloatType{0.5} * weights.dot(f_err.colwise().squaredNorm()),
                        f_err_norm);
}

//------------------------------------------------------------------------------
std::pair<FloatType, VectorX> evaluateUnitPlaneErrors(
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

  // Compute difference on unit plane.
  Keypoints uv_err = project2Vectorized(p_C) - data.uv;
  VectorX uv_err_norm = uv_err.colwise().norm();

  // Account that features at higher levels have higher uncertainty.
  uv_err_norm.array() /= data.scale.array();

  // At the first iteration, compute the scale of the error.
  if (first_iteration)
  {
    data.measurement_sigma =
        PoseOptimizer::ScaleEstimator::compute(uv_err_norm);
  }

  // Robust cost function.
  VectorX weights = PoseOptimizer::WeightFunction::weightVectorized(
                      uv_err_norm.array() / data.measurement_sigma);

  // Instead of whitening the error and the Jacobian, we apply sigma to the weights:
  weights.array() /= (data.scale.array() * data.measurement_sigma * data.measurement_sigma);

  if (H && g)
  {
    const Matrix3 R_C_W = T_C_W.getRotationMatrix();
    const int n = data.f.cols();
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3;
    for (int i = 0; i < n; ++i)
    {
      // Jacobian computation.
      G.block<3,3>(0,3) = -skewSymmetric(data.p_W.col(i));
      Matrix23 J_proj = dUv_dLandmark(p_C.col(i));
      Matrix26 J = J_proj * R_C_W * G;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * uv_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return std::make_pair(FloatType{0.5} * weights.dot(uv_err.colwise().squaredNorm()),
                        uv_err_norm);
}

//------------------------------------------------------------------------------
std::pair<FloatType, VectorX> evaluateLineErrors(
    const Transformation& T_B_W,
    const bool first_iteration,
    PoseOptimizerFrameData& data,
    PoseOptimizer::HessianMatrix* H,
    PoseOptimizer::GradientVector* g)
{
  const Transformation T_C_W = data.T_C_B * T_B_W;
  const Matrix3 R_C_W = T_C_W.getRotationMatrix();
  const Vector3 camera_pos_W = T_C_W.inverse().getPosition();
  // Compute error.
  const Matrix3X line_measurements_W = R_C_W.transpose() * data.line_measurements_C;
  const size_t n = data.line_measurements_C.cols();
  Matrix2X error(2, n);
  for (size_t i = 0; i < n; ++i)
  {
    const Vector3 direction_W = data.lines_W[i].direction();
    error(0, i) = line_measurements_W.col(i).dot(direction_W);
    const Vector3 camera_to_anchor = camera_pos_W - data.lines_W[i].anchorPoint();
    error(1, i) =
        line_measurements_W.col(i).dot(camera_to_anchor) /
        camera_to_anchor.norm();
  }
  VectorX error_norm = error.colwise().norm();

  // At the first iteration, compute the scale of the error.
  if (first_iteration)
  {
    data.measurement_sigma = PoseOptimizer::ScaleEstimator::compute(error_norm);
  }

  // Robust cost function.
  VectorX weights(n);
  weights.setOnes();

  // Instead of whitening the error and the Jacobian, we apply sigma to the weights:
  // weights.array() /= (data.measurement_sigma * data.measurement_sigma);

  if (H && g)
  {
    for (size_t i = 0; i < n; ++i)
    {
      // Jacobian computation.
      Matrix26 J = dLineMeasurement_dPose(T_B_W, data.T_C_B,
                                          line_measurements_W.col(i),
                                          data.lines_W[i].anchorPoint(),
                                          data.lines_W[i].direction());

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * error.col(i) * weights(i);
    }
  }

  return std::make_pair(FloatType{0.5} * weights.dot(error.colwise().squaredNorm()),
                        error_norm);
}

//------------------------------------------------------------------------------
std::vector<KeypointIndex> getOutlierIndices(
    PoseOptimizerFrameData& data,
    const Camera& cam,
    const Transformation& T_B_W,
    const FloatType pixel_threshold)
{
  FloatType chi2;
  VectorX err_norm_vec;
  FloatType error_multiplier = 1.0;
  FloatType threshold = pixel_threshold; //! @todo: multiple thresholds for multiple residual blocks!
  switch (data.type)
  {
    case PoseOptimizerResidualType::Bearing:
      std::tie(chi2, err_norm_vec) =
          evaluateBearingErrors(T_B_W, false, data, nullptr, nullptr);
      //! @todo: check Zichao's threshold.
      threshold =
          std::abs(2.0 * std::sin(0.5*cam.getApproxBearingAngleFromPixelDifference(pixel_threshold)));
      break;
    case PoseOptimizerResidualType::UnitPlane:
      std::tie(chi2, err_norm_vec) =
          evaluateUnitPlaneErrors(T_B_W, false, data, nullptr, nullptr);
      error_multiplier = 1.0 / std::abs(cam.projectionParameters()(0));
      threshold = pixel_threshold * error_multiplier;
      break;
    default:
      LOG(FATAL) << "Residual type not implemented.";
      break;
  }

  std::vector<KeypointIndex> outliers;
  outliers.reserve(err_norm_vec.size() / 2);
  VLOG(100) << "Reproj. threshold = " << threshold;
  for (int i = 0; i < err_norm_vec.size(); ++i)
  {
    if (err_norm_vec(i) > threshold)
    {
      VLOG(100) << "Outlier: Reproj. error = " << err_norm_vec(i) / error_multiplier << "px";
      outliers.push_back(data.kp_idx(i));
    }
  }
  return outliers;
}

} // namespace ze


