#include <ze/imu_evaluation/pre_integrator_base.hpp>

namespace ze {

//------------------------------------------------------------------------------
PreIntegrator::PreIntegrator(
    Matrix3 gyro_noise_covariance,
    IntegratorType integrator_type)
  : gyro_noise_covariance_(gyro_noise_covariance)
  , integrator_type_(integrator_type)
  , compute_absolutes_(false)
{
  R_i_j_.push_back(Matrix3::Identity());
  D_R_i_j_.push_back(Matrix3::Identity());
  covariance_i_j_.push_back(Matrix3::Zero());

  R_i_k_.push_back(Matrix3::Identity());
  D_R_i_k_.push_back(Matrix3::Identity());
  covariance_i_k_.push_back(Matrix3::Zero());
}

//------------------------------------------------------------------------------
void PreIntegrator::setInitialOrientation(Matrix3 initial_orientation)
{
  R_i_j_.clear();
  R_i_k_.clear();
  R_i_j_.push_back(initial_orientation);
  R_i_k_.push_back(initial_orientation);
}

//------------------------------------------------------------------------------
void PreIntegrator::pushD_R_i_j(times_container_t imu_stamps,
                                measurements_container_t imu_measurements)
{
  CHECK_EQ(static_cast<int>(imu_stamps.size()), imu_measurements.cols());

  // Append the new measurements to the container,
  measurements_.resize(6, measurements_.cols() + imu_measurements.cols());
  measurements_.rightCols(imu_measurements.cols()) = imu_measurements.leftCols(
                                                       imu_measurements.cols());

  integrate(imu_stamps, imu_measurements);

  // Sanity checks:
  if (compute_absolutes_)
  {
    CHECK_EQ(D_R_i_k_.size(), R_i_k_.size());
    CHECK_EQ(D_R_i_j_.size(), R_i_j_.size());
  }

  CHECK_EQ(D_R_i_k_.size(), covariance_i_k_.size());
  CHECK_EQ(D_R_i_k_.size(), times_raw_.size());
  CHECK_EQ(static_cast<int>(times_raw_.size()), measurements_.cols());

  CHECK_EQ(D_R_i_j_.size(), covariance_i_j_.size());
  CHECK_EQ(D_R_i_j_.size(), times_.size());
}

} // namespace ze
