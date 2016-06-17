#include <ze/imu_evaluation/manifold_pre_integrator.hpp>

namespace ze {

//------------------------------------------------------------------------------
ManifoldPreIntegrationState::ManifoldPreIntegrationState(
    Matrix3 gyro_noise_covariance,
    PreIntegrator::IntegratorType integrator_type,
    const preintegrated_orientation_container_t* D_R_i_k_reference)
  : PreIntegrator(gyro_noise_covariance, integrator_type)
  , D_R_i_k_reference_(D_R_i_k_reference)
{
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::doPushD_R_i_j(
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

  // Explicitly push beginning if times_ is empty.
  if (times_.size() == 0)
  {
    times_.push_back(times_raw_.front());
  }

  times_.push_back(times_raw_.back());

  // Push the keyframe sampled pre-integration states:
  D_R_i_j_.push_back(D_R_i_k_.back());
  R_i_j_.push_back(R_i_j_.back() * D_R_i_j_.back());

  // push covariance
  covariance_i_j_.push_back(covariance_i_k_.back());
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::doPushFirstOrderFwd(
    times_container_t stamps,
    measurements_container_t measurements)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    FloatType dt = stamps[i+1] - stamps[i];

    Vector6 measurement = measurements.col(i);
    Vector3 gyro_measurement = measurement.tail<3>(3);


    // Reset to 0 at every step:
    if (i == 0)
    {
      pushInitialValuesFwd(dt, gyro_measurement);
    }
    else
    {
      pushPreIntegrationStepFwd(dt, gyro_measurement);
    }

    times_raw_.push_back(stamps[i]);
  }
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::doPushFirstOrderMid(
    times_container_t stamps,
    measurements_container_t measurements)
{
  // Integrate measurements between frames.
  for (int i = 0; i < measurements.cols() - 1; ++i)
  {
    FloatType dt = stamps[i+1] - stamps[i];

    Vector6 measurement = measurements.col(i);
    Vector3 gyro_measurement = measurement.tail<3>(3);
    Vector3 gyro_measurement2 = measurements.col(i+1).tail<3>(3);


    // Reset to 0 at every step:
    if (i == 0)
    {
      pushInitialValuesMid(dt, gyro_measurement, gyro_measurement2);
    }
    else
    {
      pushPreIntegrationStepMid(dt, gyro_measurement, gyro_measurement2);
    }

    times_raw_.push_back(stamps[i]);
  }
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::pushInitialValuesFwd(
    const FloatType dt,
    const Eigen::Ref<Vector3>& gyro_measurement)
{
  Matrix3 increment = Quaternion::exp(gyro_measurement * dt).getRotationMatrix();

  // D_R_i_k restarts with every push to the container.
  D_R_i_k_.push_back(Matrix3::Identity());
  R_i_k_.push_back(R_i_k_.back() * increment);
  covariance_i_k_.push_back(Matrix3::Zero());
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::pushPreIntegrationStepFwd(
    const FloatType dt,
    const Eigen::Ref<Vector3>& gyro_measurement)
{
  Matrix3 increment = Quaternion::exp(gyro_measurement * dt).getRotationMatrix();

  D_R_i_k_.push_back(D_R_i_k_.back() * increment);
  R_i_k_.push_back(R_i_k_.back() * increment);

  // Propagate Covariance:
  Matrix3 J_r = expmapDerivativeSO3(gyro_measurement * dt);

  // Covariance of the discrete process.
  Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;

  Matrix3 D_R_i_k;
  if (D_R_i_k_reference_)
  {
    D_R_i_k = (*D_R_i_k_reference_)[D_R_i_k_.size() - 1];
  }
  else
  {
    D_R_i_k = D_R_i_k_.back();
  }

  covariance_i_k_.push_back(
        D_R_i_k.transpose() * covariance_i_k_.back() * D_R_i_k
        + J_r * gyro_noise_covariance_d * dt * dt * J_r.transpose());
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::pushInitialValuesMid(
    const FloatType dt,
    const Eigen::Ref<Vector3>& gyro_measurement,
    const Eigen::Ref<Vector3>& gyro_measurement2)
{
  Matrix3 increment = Quaternion::exp((gyro_measurement + gyro_measurement2) * 0.5 * dt).getRotationMatrix();

  // D_R_i_k restarts with every push to the container.
  D_R_i_k_.push_back(Matrix3::Identity());
  R_i_k_.push_back(R_i_k_.back() * increment);
  covariance_i_k_.push_back(Matrix3::Zero());
}

//------------------------------------------------------------------------------
void ManifoldPreIntegrationState::pushPreIntegrationStepMid(
    const FloatType dt,
    const Eigen::Ref<Vector3>& gyro_measurement,
    const Eigen::Ref<Vector3>& gyro_measurement2)
{
  Matrix3 increment = Quaternion::exp(
                        (gyro_measurement + gyro_measurement2) * 0.5 * dt)
                      .getRotationMatrix();

  D_R_i_k_.push_back(D_R_i_k_.back() * increment);
  R_i_k_.push_back(R_i_k_.back() * increment);

  // Propagate Covariance:
  Matrix3 J_r = expmapDerivativeSO3(gyro_measurement * dt);

  // Covariance of the discrete process.
  Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;

  Matrix3 D_R_i_k;
  if (D_R_i_k_reference_)
  {
    D_R_i_k = (*D_R_i_k_reference_)[D_R_i_k_.size() - 1];
  }
  else
  {
    D_R_i_k = D_R_i_k_.back();
  }

  covariance_i_k_.push_back(
        D_R_i_k.transpose() * covariance_i_k_.back() * D_R_i_k
        + J_r * gyro_noise_covariance_d * dt * dt * J_r.transpose());
}

} // namespace ze
