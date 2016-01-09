#include <ze/geometry/pose_optimizer.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <glog/logging.h>

#include <ze/common/stl_utils.h>

namespace ze {

PoseOptimizer::PoseOptimizer(const Keypoints& measurements,
                             const Positions& landmarks,
                             const Transformation& T_C_B,
                             const PinholeCamera& cam,
                             double pixel_measurement_sigma)
    : px_(measurements)
    , p_W_(landmarks)
    , T_C_B_(T_C_B)
    , cam_(cam)
    , px_measurement_sigma_(pixel_measurement_sigma)
  {}

double PoseOptimizer::evaluateError(
    const Transformation&T_B_W, HessianMatrix* H, GradientVector* g)
{
  // Transform points from world coordinates to camera coordinates.
  Positions p_C = (T_C_B_ * T_B_W).transformVectorized(p_W_);

  // Normalize points to obtain estimated bearing vectors.
  Keypoints px_est = cam_.projectVectorized(p_C);

  // Compute difference between bearing vectors.
  Keypoints px_err = px_est - px_;

  Eigen::VectorXd px_err_norm = px_err.colwise().norm();

  // Robust cost function:
  Eigen::VectorXd weights =
      WeightFunction::weightVectorized(px_err_norm / px_measurement_sigma_);

  // Whiten error:
  px_err /= px_measurement_sigma_;


  //
  // TODO(cfo): This can be implemented more efficiently...
  //

  if(H && g)
  {
    // Projection Jacobian transposed:
    //Eigen::Matrix<double, 6, Eigen::Dynamic> J_proj_tr =
    //    cam_.dProject_dBearingVectorized(p_C);

    Eigen::Matrix3d R_C_B = T_C_B_.getRotation().getRotationMatrix();
    for(int i = 0; i < px_.cols(); ++i)
    {
      // Jacobian computation.
      Eigen::Matrix<double, 3, 6> G_x;
      G_x.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      G_x.block<3,3>(0,3) = - skewSymmetric(p_W_.col(i));

      Eigen::Matrix<double, 2, 3> J_cam = cam_.dProject_dBearing(p_C.col(i));
      Eigen::Matrix<double, 2, 6> J =
          //Eigen::Map<Eigen::Matrix<double, 3, 2>>(J_proj_tr.col(i).data()).transpose()
          J_cam * R_C_B * T_B_W.getRotationMatrix() * G_x; // TODO: can use property of skew symmetric, to use point in camera coordinates!

      // Whiten Jacobian:
      J /= px_measurement_sigma_;

      // Compute Hessian and Gradient Vector:
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * px_err.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  return  0.5 * weights.dot(px_err.colwise().squaredNorm());
}

void PoseOptimizer::update(
    const Transformation& T_Bold_W, const UpdateVector& dx,
    Transformation& T_Bnew_W)
{
  T_Bnew_W = T_Bold_W * Transformation::exp(dx);
}

} // namespace ze


