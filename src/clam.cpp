#include <ze/geometry/clam.h>
#include <ze/geometry/pose_optimizer.h>
#include <ze/geometry/pose_prior.h>

namespace ze {

Clam::Clam(
    const ClamLandmarks& landmarks,
    const std::vector<ClamFrameData>& data,
    const CameraRig& rig,
    const Transformation& T_Bc_Br_prior,
    const FloatType prior_weight_pos,
    const FloatType prior_weight_rot)
  : landmarks_(landmarks)
  , data_(data)
  , rig_(rig)
  , T_Bc_Br_prior_(T_Bc_Br_prior)
  , prior_weight_pos_(prior_weight_pos)
  , prior_weight_rot_(prior_weight_rot)
{}

FloatType Clam::evaluateError(
    const ClamState& state, HessianMatrix* H, GradientVector* g)
{
  CHECK_EQ(data_.size(), measurement_sigma_.size());
  FloatType chi2 = 0.0;

  const Transformation& T_Bc_Br = state.at<0>();
  const VectorX& inv_depth = state.at<1>();

  // ---------------------------------------------------------------------------
  // Localization

  // Loop over all cameras in rig.
  for (size_t i = 0; i < data_.size(); ++i)
  {
    const ClamFrameData& data = data_[i];
    FloatType& measurement_sigma = measurement_sigma_[i];

    // Transform points from reference coordinates to camera coordinates.
    const Transformation T_C_Br = data.T_C_B * T_Bc_Br; //! @todo(cfo): use inverse-depth coordinates!
    const Positions p_C = T_C_Br.transformVectorized(data.p_Br);

    // Normalize points to obtain estimated bearing vectors.
    Bearings f_est = p_C;
    normalizeBearings(f_est);

    // Compute difference between bearing vectors.
    Bearings f_err = f_est - data.f_C;
    const VectorX f_err_norm = f_err.colwise().norm();

    // At the first iteration, compute the scale of the error.
    if(iter_ == 0)
    {
      measurement_sigma = ScaleEstimator::compute(f_err_norm);
      VLOG(300) << "Cam " << i << " measurement sigma = " << measurement_sigma;
    }

    // Robust cost function.
    const VectorX weights =
        WeightFunction::weightVectorized(f_err_norm / measurement_sigma);

    // Whiten error.
    f_err /= measurement_sigma;

    if (H && g)
    {
      const Matrix3 R_C_Br = T_C_Br.getRotationMatrix();
      const int n = data.f_C.cols();
      Matrix36 G;
      G.block<3,3>(0,0) = I_3x3;
      for (int i = 0; i < n; ++i)
      {
        // Jacobian computation.
        G.block<3,3>(0,3) = - skewSymmetric(data.p_Br.col(i));
        Matrix3 J_normalization = dBearing_dLandmark(p_C.col(i));
        Matrix36 J = J_normalization * R_C_Br * G;

        // Whiten Jacobian.
        J /= measurement_sigma;

        // Compute Hessian and Gradient Vector.
        H->topLeftCorner<6,6>().noalias() += J.transpose() * J * weights(i);
        g->head<6>().noalias() -= J.transpose() * f_err.col(i) * weights(i);
      }
    }

    // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
    chi2 += 0.5 * weights.dot(f_err.colwise().squaredNorm());
  }

  // ---------------------------------------------------------------------------
  // Mapping
  for (size_t i = 0; i < data_.size(); ++i)
  {
    const ClamFrameData& data = data_[i];
    const Camera& cam = rig_.at(i);
    const Transformation T_C_Br = data.T_C_B * T_Bc_Br;
    for (const std::pair<uint32_t, Keypoint>& m : data.landmark_measurements)
    {
      HomPosition p_Br_h;
      p_Br_h.head<3>() = landmarks_.f_Br.col(m.first)
          + landmarks_.origin_Br.col(m.first) * inv_depth(m.first);
      p_Br_h(3) = inv_depth(m.first);
      const HomPosition p_C_h = T_C_Br.transform4(p_Br_h);
      const Keypoint px_est = cam.project(p_C_h.head<3>());
      const Keypoint px_err = px_est - m.second;

      const Matrix23 J_proj = cam.dProject_dLandmark(p_C_h.head<3>());





    }
  }

  // ---------------------------------------------------------------------------
  // Prior
  if (prior_weight_rot_ > 0.0f || prior_weight_pos_ > 0.0f)
  {
    applyPosePrior(
          T_Bc_Br, T_Bc_Br_prior_, prior_weight_rot_, prior_weight_pos_,
          H->block<6,6>(0,0), g->segment<6>(0));
  }
  return chi2;
}

} // namespace ze
