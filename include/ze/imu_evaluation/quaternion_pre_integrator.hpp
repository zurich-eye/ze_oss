#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/common/transformation.h>
#include <ze/imu_evaluation/scenario_runner.hpp>

namespace ze {

//! A class that represents all quaternion pre-integrated states.
//! The class operates on pure-quaternions but stores a copy of the estimates
//! are Matrix3 in the base class structure. This is not efficient but offers
//! nice and unfied interfaces.
class QuaternionPreIntegrationState : public PreIntegrator
{
public:
  typedef std::vector<Quaternion> preintegrated_orientation_quat_container_t;

  QuaternionPreIntegrationState(
      Matrix3 gyro_noise_covariance)
    : PreIntegrator(gyro_noise_covariance)
  {
    D_R_i_j_quat_.push_back(Quaternion());
    R_i_j_quat_.push_back(Quaternion());
    D_R_i_k_quat_.push_back(Quaternion());
    R_i_k_quat_.push_back(Quaternion());
  }

  void pushD_R_i_j(times_container_t stamps,
                   measurements_container_t measurements)
  {
    // Append the new measurements to the container,
    measurements_.resize(6, measurements_.cols() + measurements.cols() - 1);
    measurements_.rightCols(measurements.cols() - 1) = measurements.leftCols(
                                                         measurements.cols() - 1);

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
        covariance_i_k_.push_back(Matrix3::Zero());
      }
      else
      {
        // Integrate
        D_R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                  D_R_i_k_quat_.back(),
                                  measurements.col(i).tail<3>(3),
                                  dt));

        R_i_k_quat_.push_back(integrateFirstOrderFwd(
                                R_i_k_quat_.back(),
                                measurements.col(i).tail<3>(3),
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

    //// Update the steps:
    times_.push_back(times_raw_.back());

    // Push the keyframe sampled pre-integration states:
    D_R_i_j_quat_.push_back(D_R_i_k_quat_.back());
    R_i_j_quat_.push_back(R_i_j_quat_.back() * D_R_i_j_quat_.back());
    D_R_i_j_.push_back(D_R_i_j_quat_.back().getRotationMatrix());
    R_i_j_.push_back(R_i_j_quat_.back().getRotationMatrix());

    // push covariance
    covariance_i_j_.push_back(covariance_i_k_.back());
  }

private:
  preintegrated_orientation_quat_container_t D_R_i_j_quat_;
  preintegrated_orientation_quat_container_t R_i_j_quat_;
  preintegrated_orientation_quat_container_t D_R_i_k_quat_;
  preintegrated_orientation_quat_container_t R_i_k_quat_;

  //! Forward integration.
  Quaternion integrateFirstOrderFwd(Quaternion q,
                                    Vector3 w_i,
                                    FloatType dt)
  {
    return q * Quaternion(Vector3(w_i * dt));
  }

  //! Midpoint integration.
  Quaternion integrateFirstOrderMid(Quaternion q,
                                    Vector3 w_i,
                                    Vector3 w_i_1,
                                    FloatType dt)
  {
    return q * Quaternion(Vector3((w_i + w_i_1) * 0.5 * dt));
  }
};

//! The factory for quaternion preintegrators
class QuaternionPreIntegrationFactory: public PreIntegratorFactory
{
public:
  typedef QuaternionPreIntegrationState::preintegrated_orientation_container_t preintegrated_orientation_container_t;

  //! Optionally provide the reference (non-corrupted) pre-integrated orientation
  //! to inject into the created manifold preintegrators.
  QuaternionPreIntegrationFactory(Matrix3 gyro_noise_covariance)
    : PreIntegratorFactory(gyro_noise_covariance)
  {
  }

  PreIntegrator::Ptr get()
  {
    return std::make_shared<QuaternionPreIntegrationState>(gyro_noise_covariance_);
  }
};

} // namespace ze
