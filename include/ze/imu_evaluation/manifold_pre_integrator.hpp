#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/common/transformation.h>
#include <ze/imu_evaluation/scenario_runner.hpp>

namespace ze {

//! A class that represents all manifold pre-integrated orientations and
//! the corresponding propagated covariance.
class ManifoldPreIntegrationState : public PreIntegrator
{
public:
  ManifoldPreIntegrationState(Matrix3 gyro_noise_covariance)
    : PreIntegrator(gyro_noise_covariance)
  {
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
      // Covariance of the discrete process.
      Matrix3 gyro_noise_covariance_d = gyro_noise_covariance_ / dt;

      Vector6 measurement = measurements.col(i);
      Vector3 gyro_measurement = measurement.tail<3>(3);

      Matrix3 increment = Quaternion::exp(gyro_measurement * dt).getRotationMatrix();

      // Reset to 0 at every step:
      if (i == 0)
      {
        // D_R_i_k restarts with every push to the container.
        D_R_i_k_.push_back(Matrix3::Identity());
        R_i_k_.push_back(R_i_k_.back() * increment);
        covariance_i_k_.push_back(Matrix3::Zero());
      }
      else
      {
        D_R_i_k_.push_back(D_R_i_k_.back() * increment);
        R_i_k_.push_back(R_i_k_.back() * increment);

        // Propagate Covariance:
        Matrix3 J_r = expmapDerivativeSO3(gyro_measurement * dt);

        covariance_i_k_.push_back(
              D_R_i_k_.back().transpose() * covariance_i_k_.back() * D_R_i_k_.back()
              + J_r * gyro_noise_covariance_d * dt * dt * J_r.transpose());
      }

      times_raw_.push_back(stamps[i]);
    }

    // Push the keyframe sampled pre-integration states:
    D_R_i_j_.push_back(D_R_i_k_.back());
    R_i_j_.push_back(R_i_j_.back() * D_R_i_j_.back());

    // push covariance
    covariance_i_j_.push_back(covariance_i_k_.back());

    times_.push_back(times_raw_.back());

  }
};

} // namespace ze
