#pragma once

#include <ze/common/transformation.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_impl.h>
#include <ze/cameras/camera_rig.h>
#include <ze/geometry/robust_cost.h>
#include <ze/geometry/lsq_solver.h>
#include <ze/geometry/lsq_state.h>

namespace ze {

using ClamState = State<Transformation, VectorX>;

struct ClamLandmarks
{
  Positions origin_Br;
  Bearings f_Br;
};

//! Data required by Clam per frame.
struct ClamFrameData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //!@ Localization data
  //!{
  //! Measurements: Bearing vectors.
  Bearings f_C;

  //! Landmark positions expressed in body of reference frame.
  //! Each column corresponds to a bearing measurement.
  Positions p_Br;

  //!}

  //!@ Mapping data
  //!{
  //! Landmark measurements {ClamLandmarks-Index, Keypoint}.
  std::vector<std::pair<uint32_t, Keypoint>> landmark_measurements;
  //!}

  //! Extrinsic transformation between camera and body (i.e., imu) frame.
  Transformation T_C_B;
};

//! Coupled localization and mapping (Clam).
class Clam : public LeastSquaresSolver<ClamState, Clam>
{
public:
  using LeastSquaresSolver::HessianMatrix;
  using LeastSquaresSolver::GradientVector;
  using ScaleEstimator = MADScaleEstimator<FloatType>;
  using WeightFunction = TukeyWeightFunction<FloatType>;

  Clam(
      const ClamLandmarks& landmarks,
      const std::vector<ClamFrameData>& data,
      const CameraRig& rig,
      const Transformation& T_Bc_Br_prior,
      const FloatType prior_weight_pos,
      const FloatType prior_weight_rot);

  FloatType evaluateError(
      const ClamState& state,
      HessianMatrix* H,
      GradientVector* g);

private:
  const ClamLandmarks& landmarks_;
  const std::vector<ClamFrameData>& data_;
  const CameraRig& rig_;
  std::vector<FloatType> measurement_sigma_localization_;
  FloatType measurement_sigma_mapping_ = 2.0;

  // Prior:
  const Transformation& T_Bc_Br_prior_;
  FloatType prior_weight_pos_;
  FloatType prior_weight_rot_;
};

inline Vector2 reprojectionResidual(
    const Eigen::Ref<const Bearing>& f_Br, //!< Bearing vector in reference body frame (Br).
    const Eigen::Ref<const Position>& p_Br, //!< Reference camera center pos in Br.
    const Camera& cam,
    const Transformation& T_C_B,
    const Transformation& T_Bc_Br,
    const FloatType inv_depth,
    const Eigen::Ref<const Keypoint>& px_measured,
    Matrix26* H1 = nullptr, Matrix21* H2 = nullptr)
{
  HomPosition p_Br_h;
  p_Br_h.head<3>() = f_Br + p_Br * inv_depth;
  p_Br_h(3) = inv_depth;
  const Transformation T_C_Br = T_C_B * T_Bc_Br;
  const HomPosition p_C_h = T_C_Br.transform4(p_Br_h);
  const Keypoint px_est = cam.project(p_C_h.head<3>());
  const Vector2 px_err = px_est - px_measured;

  if(H1 || H2)
  {
    // H1 = dPx / dT_Bc_Br
    Matrix23 J_proj = cam.dProject_dLandmark(p_C_h.head<3>());
    if(inv_depth < 0.0)
    {
      J_proj *= -1.0;
    }
    Matrix36 G;
    G.block<3,3>(0,0) = I_3x3 * inv_depth; // translation
    G.block<3,3>(0,3) = - skewSymmetric(p_Br_h.head<3>()); // rotation
    *H1 = J_proj * T_C_Br.getRotationMatrix() * G;

    // H2 = dPx / dinv_depth
    *H2 = J_proj * (T_C_Br.getRotation().rotate(p_Br) + T_C_Br.getPosition());
  }

  return px_err;
}

} // namespace ze
