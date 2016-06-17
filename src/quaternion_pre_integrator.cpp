#include <ze/imu_evaluation/quaternion_pre_integrator.hpp>

namespace ze {

//------------------------------------------------------------------------------
QuaternionPreIntegrationState::QuaternionPreIntegrationState(
    Matrix3 gyro_noise_covariance,
    PreIntegrator::IntegratorType integrator_type)
  : PreIntegrator(gyro_noise_covariance, integrator_type)
{
  D_R_i_j_quat_.push_back(Quaternion());
  R_i_j_quat_.push_back(Quaternion());
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::setInitialOrientation(
                                      Matrix3 initial_orientation)
{
  PreIntegrator::setInitialOrientation(initial_orientation);
  R_i_j_quat_.push_back(Quaternion(initial_orientation));
  R_i_k_quat_.push_back(Quaternion(initial_orientation));
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushD_R_i_j(
    times_container_t stamps,
    measurements_container_t measurements)
{

  switch (integrator_type_)
  {
    case PreIntegrator::FirstOrderForward:
      doPushFirstOrderFwd(stamps, measurements);
      break;
    case PreIntegrator::FirstOrderMidward:
      doPushFirstOrderMid(stamps, measurements);
      break;
    default:
      throw std::runtime_error("No valid integrator supplied");
  }

  times_raw_.push_back(stamps.back());

  //// Update the steps:
  // Explicitly push beginning if times_ is empty.
  if (times_.size() == 0)
  {
    times_.push_back(times_raw_.front());
  }
  times_.push_back(times_raw_.back());

  // Push the keyframe sampled pre-integration states:
  D_R_i_j_quat_.push_back(D_R_i_k_quat_.back());
  R_i_j_quat_.push_back(R_i_j_quat_.back() * D_R_i_j_quat_.back());
  D_R_i_j_.push_back(D_R_i_j_quat_.back().getRotationMatrix());
  R_i_j_.push_back(R_i_j_quat_.back().getRotationMatrix());

  // push covariance
  covariance_i_j_.push_back(covariance_i_k_.back());
}

//------------------------------------------------------------------------------
Quaternion QuaternionPreIntegrationState::integrateFirstOrderFwd(Quaternion q,
                                                                 Vector3 w_i,
                                                                 FloatType dt)
{
  return q * Quaternion(Vector3(w_i * dt));
}

//------------------------------------------------------------------------------
Quaternion QuaternionPreIntegrationState::integrateFirstOrderMid(Quaternion q,
                                                                 Vector3 w_i,
                                                                 Vector3 w_i_1,
                                                                 FloatType dt)
{
  return q * Quaternion(Vector3((w_i + w_i_1) * 0.5 * dt));
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushFirstOrderFwd(times_container_t stamps,
                         measurements_container_t measurements)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());
      R_i_k_quat_.push_back(integrateFirstOrderFwd(
                              R_i_k_quat_.back(),
                              measurements.col(i).tail<3>(3),
                              dt));
      D_R_i_k_.push_back(Matrix3::Identity());
      R_i_k_.push_back(R_i_k_quat_.back().getRotationMatrix());
      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate

      switch (integrator_type_)
      {
        case FirstOrderForward:
          D_R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                    D_R_i_k_quat_.back(),
                                    measurements.col(i).tail<3>(3),
                                    dt));

          R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                  R_i_k_quat_.back(),
                                  measurements.col(i).tail<3>(3),
                                  dt));
          break;
        case FirstOrderMidward:
          D_R_i_k_quat_.push_back(integrateFirstOrderMid(
                                    D_R_i_k_quat_.back(),
                                    measurements.col(i).tail<3>(3),
                                    measurements.col(i + 1).tail<3>(3),
                                    dt));

          R_i_k_quat_.push_back(integrateFirstOrderMid(
                                  R_i_k_quat_.back(),
                                  measurements.col(i).tail<3>(3),
                                  measurements.col(i + 1).tail<3>(3),
                                  dt));
          break;

        default:
          throw std::runtime_error("No valid integrator supplied");
      }

      // Push the rotation matrix equivalent representations:
      D_R_i_k_.push_back(D_R_i_k_quat_.back().getRotationMatrix());
      R_i_k_.push_back(R_i_k_quat_.back().getRotationMatrix());

      // Covariance Prediction (FWD Integrated)
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;
      Matrix3 D_R = Quaternion(Vector3(measurements.col(i).tail<3>(3) * dt)).
                    getRotationMatrix();
      covariance_i_k_.push_back(
            D_R.transpose() * covariance_i_k_.back() * D_R
            + gyro_noise_covariance_d);
    }

    times_raw_.push_back(stamps[i]);
  }
}

//------------------------------------------------------------------------------
void QuaternionPreIntegrationState::doPushFirstOrderMid(
    times_container_t stamps,
    measurements_container_t measurements)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    FloatType dt = stamps[i+1] - stamps[i];

    // Reset to 0 at every step:
    if (i == 0)
    {
      D_R_i_k_quat_.push_back(Quaternion());
      R_i_k_quat_.push_back(integrateFirstOrderMid(
                              R_i_k_quat_.back(),
                              measurements.col(i).tail<3>(3),
                              measurements.col(i + 1).tail<3>(3),
                              dt));
      D_R_i_k_.push_back(Matrix3::Identity());
      R_i_k_.push_back(R_i_k_quat_.back().getRotationMatrix());
      covariance_i_k_.push_back(Matrix3::Zero());
    }
    else
    {
      // Integrate
      D_R_i_k_quat_.push_back(integrateFirstOrderMid(
                                D_R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
                                measurements.col(i + 1).tail<3>(3),
                                dt));

      R_i_k_quat_.push_back(integrateFirstOrderMid(
                              R_i_k_quat_.back(),
                              measurements.col(i).tail<3>(3),
                              measurements.col(i + 1).tail<3>(3),
                              dt));

      // Push the rotation matrix equivalent representations:
      D_R_i_k_.push_back(D_R_i_k_quat_.back().getRotationMatrix());
      R_i_k_.push_back(R_i_k_quat_.back().getRotationMatrix());

      // Covariance Prediction (FWD Integrated)
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;
      Matrix3 D_R = Quaternion(Vector3(measurements.col(i).tail<3>(3) * dt)).
                    getRotationMatrix();
      covariance_i_k_.push_back(
            D_R.transpose() * covariance_i_k_.back() * D_R
            + gyro_noise_covariance_d);
    }

    times_raw_.push_back(stamps[i]);
  }
}

} // namespace ze
