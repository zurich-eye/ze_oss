#include <ze/geometry/pose_optimizer.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>
#include <ze/geometry/jacobians.h>

namespace ze {

PoseOptimizerBearingVectors::PoseOptimizerBearingVectors(
    const Bearings& measurements, const Positions& landmarks,
    const Transformation& T_C_B, double measurement_sigma)
    : f_(measurements)
    , p_W_(landmarks)
    , T_C_B_(T_C_B)
    , measurement_sigma_(measurement_sigma)
{}

double PoseOptimizerBearingVectors::evaluateError(
    const Transformation& T_B_W, HessianMatrix* H, GradientVector* g)
{
  // Transform points from world coordinates to camera coordinates.
  const Transformation T_C_W = T_C_B_ * T_B_W;
  Positions p_C = T_C_W.transformVectorized(p_W_);

  // Normalize points to obtain estimated bearing vectors.
  Bearings f_est = p_C;
  normalizeBearings(f_est);

  // Compute difference between bearing vectors.
  Bearings f_err = f_est - f_;

  Eigen::VectorXd f_err_norm = f_err.colwise().norm();

  // Robust cost function.
  Eigen::VectorXd weights =
      WeightFunction::weightVectorized(f_err_norm / measurement_sigma_);

  // Whiten error.
  f_err /= measurement_sigma_;

  if(H && g)
  {
    const Eigen::Matrix3d R_C_W = T_C_W.getRotationMatrix();
    const int n = f_.cols();
    for(int i = 0; i < n; ++i)
    {
      // Jacobian computation.
      Eigen::Matrix<double, 3, 6> G;
      G.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      G.block<3,3>(0,3) = - skewSymmetric(p_W_.col(i));
      Eigen::Matrix3d J_normalization = dBearing_dLandmark(p_C.col(i));
      Eigen::Matrix<double, 3, 6> J = J_normalization * R_C_W * G;

      // Whiten Jacobian.
      J /= measurement_sigma_;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * f_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return  0.5 * weights.dot(f_err.colwise().squaredNorm());
}

void PoseOptimizerBearingVectors::update(
    const Transformation& T_Bold_W, const UpdateVector& dx,
    Transformation& T_Bnew_W)
{
  T_Bnew_W = T_Bold_W * Transformation::exp(dx);
}

} // namespace ze


