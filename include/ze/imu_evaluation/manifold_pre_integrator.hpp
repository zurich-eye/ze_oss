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
  //! Set a preintegrated container with reference values for the relative
  //! orientation to use instead of the corrupted pre-integrated rotations
  //! within the container (for covariance propagation).
  ManifoldPreIntegrationState(
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type,
      const preintegrated_orientation_container_t* D_R_i_k_reference = nullptr)
    : PreIntegrator(gyro_noise_covariance, integrator_type)
    , D_R_i_k_reference_(D_R_i_k_reference)
  {
  }

  void doPushD_R_i_j(times_container_t stamps,
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

private:
  //! The manifold pre-integrator can operate on noise-free relative
  //! orientation estimates if provided. The provided estimates have to be generated
  //! with exactly the same settings as the simulation run to guarantee
  //! index equivalence.
  const preintegrated_orientation_container_t* D_R_i_k_reference_;

  void doPushFirstOrderFwd(times_container_t stamps,
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

  void doPushFirstOrderMid(times_container_t stamps,
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

  //! Push the values used for a new pre-integration batch.
  inline void pushInitialValuesFwd(const FloatType dt,
                                   const Eigen::Ref<Vector3>& gyro_measurement)
  {
    Matrix3 increment = Quaternion::exp(gyro_measurement * dt).getRotationMatrix();

    // D_R_i_k restarts with every push to the container.
    D_R_i_k_.push_back(Matrix3::Identity());
    R_i_k_.push_back(R_i_k_.back() * increment);
    covariance_i_k_.push_back(Matrix3::Zero());
  }

  //! Push a new integration step to the containers.
  inline void pushPreIntegrationStepFwd(const FloatType dt,
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

  //! Push the values used for a new pre-integration batch.
  inline void pushInitialValuesMid(const FloatType dt,
                                   const Eigen::Ref<Vector3>& gyro_measurement,
                                   const Eigen::Ref<Vector3>& gyro_measurement2)
  {
    Matrix3 increment = Quaternion::exp((gyro_measurement + gyro_measurement2) * 0.5 * dt).getRotationMatrix();

    // D_R_i_k restarts with every push to the container.
    D_R_i_k_.push_back(Matrix3::Identity());
    R_i_k_.push_back(R_i_k_.back() * increment);
    covariance_i_k_.push_back(Matrix3::Zero());
  }

  //! Push a new integration step to the containers.
  inline void pushPreIntegrationStepMid(const FloatType dt,
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

};

//! The factory for manifold preintegrators
class ManifoldPreIntegrationFactory: public PreIntegratorFactory
{
public:
  typedef ManifoldPreIntegrationState::preintegrated_orientation_container_t preintegrated_orientation_container_t;

  //! Optionally provide the reference (non-corrupted) pre-integrated orientation
  //! to inject into the created manifold preintegrators.
  ManifoldPreIntegrationFactory(
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type,
      const preintegrated_orientation_container_t* D_R_i_k_reference = nullptr)
    : PreIntegratorFactory(gyro_noise_covariance, integrator_type)
    , D_R_i_k_reference_(D_R_i_k_reference)
  {
  }

  PreIntegrator::Ptr get()
  {
    if (D_R_i_k_reference_)
    {
      return std::make_shared<ManifoldPreIntegrationState>(
            gyro_noise_covariance_,
            integrator_type_,
            D_R_i_k_reference_);
    }
    else
    {
      return std::make_shared<ManifoldPreIntegrationState>(
            gyro_noise_covariance_,
            integrator_type_);
    }
  }

private:
  //! The optional already pre-integrated (clean, non corrupted) orientations.
  const preintegrated_orientation_container_t* D_R_i_k_reference_;
};

} // namespace ze
