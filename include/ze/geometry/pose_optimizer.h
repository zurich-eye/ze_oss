#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/geometry/least_squares_solver.h>

namespace ze {

class PoseOptimizer : public LeastSquaresSolver<6, Transformation, PoseOptimizer>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;

  PoseOptimizer(const Keypoints& measurements,
                const Positions& landmarks,
                const Transformation& T_C_B,
                const PinholeCamera& cam,
                Transformation& T_B_W)
    : u_(measurements)
    , p_W_(landmarks)
    , T_C_B_(T_C_B)
    , cam_(cam)
  {}

  double evaluateError(const Transformation& T_B_W, HessianMatrix* H,
                       GradientVector *g)
  {
    // Transform points from world coordinates to camera coordinates.
    Positions p_C = (T_C_B_ * T_B_W).transformVectorized(p_W_);

    // Normalize points to obtain estimated bearing vectors.
    Keypoints u_est = cam_.projectVectorized(p_C);

    // Compute difference between bearing vectors.
    Keypoints u_err = u_ - u_est;

    Eigen::VectorXd u_err_norm = u_err.colwise().norm();

    //
    // TODO: Implement robust cost function that takes vector of errors.
    //

    // Projection jacobian tranposed:
    Eigen::Matrix<double, 6, Eigen::Dynamic> J_proj_tr =
        cam_.dProject_dBearingVectorized(p_C);

    // Compute Jacobian w.r.t. body
    //
    // TODO
    //

    return 0.0;
  }

  void update(const Transformation& T_Bold_W, const UpdateVector& dx,
              Transformation& T_Bnew_W)
  {
    T_Bnew_W = Transformation::exp(dx)*T_Bold_W;
  }

private:
  const Keypoints& u_;  ///< Bearing vectors corresponding to feature measurements.
  const Positions& p_W_;    ///< 3D points corresponding to features.
  const Transformation& T_C_B_;   ///< Camera-IMU extrinsic calibration.
  const PinholeCamera& cam_;
};

} // namespace ze
