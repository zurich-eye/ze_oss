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
      bool useSimpleCovariancePropagation = false,
      const preintegrated_orientation_container_t* D_R_i_k_reference = nullptr);

  void doPushD_R_i_j(times_container_t stamps,
                     measurements_container_t measurements);

  ~ManifoldPreIntegrationState() {}

private:
  //! The manifold pre-integrator can operate on noise-free relative
  //! orientation estimates if provided. The provided estimates have to be generated
  //! with exactly the same settings as the simulation run to guarantee
  //! index equivalence.
  const preintegrated_orientation_container_t* D_R_i_k_reference_;

  //! Should the simple covariance propgatation be used?
  bool useSimpleCovariancePropagation_;

  void doPushFirstOrderFwd(times_container_t stamps,
                           measurements_container_t measurements);

  void doPushFirstOrderMid(times_container_t stamps,
                           measurements_container_t measurements);

  //! Push the values used for a new pre-integration batch.
  void pushInitialValuesFwd(const FloatType dt,
                            const Eigen::Ref<Vector3>& gyro_measurement);

  //! Push a new integration step to the containers.
  inline void pushPreIntegrationStepFwd(const FloatType dt,
                                 const Eigen::Ref<Vector3>& gyro_measurement);

  //! Push the values used for a new pre-integration batch.
  inline void pushInitialValuesMid(const FloatType dt,
                            const Eigen::Ref<Vector3>& gyro_measurement,
                            const Eigen::Ref<Vector3>& gyro_measurement2);


  //! Push a new integration step to the containers.
  void pushPreIntegrationStepMid(
      const FloatType dt,
      const Eigen::Ref<Vector3>& gyro_measurement,
      const Eigen::Ref<Vector3>& gyro_measurement2);

};

//------------------------------------------------------------------------------
//! The factory for manifold preintegrators
class ManifoldPreIntegrationFactory: public PreIntegratorFactory
{
public:
  typedef ManifoldPreIntegrationState::preintegrated_orientation_container_t preintegrated_orientation_container_t;

  //! Optionally provide the reference (non-corrupted) pre-integrated orientation
  //! to inject into the created manifold preintegrators.
  //! Should a simple covariance propagation be used (e.g. the Jacobian assumed
  //! close to Identity and neglected)
  ManifoldPreIntegrationFactory(
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type,
      bool useSimpleCovariancePropagation = false,
      const preintegrated_orientation_container_t* D_R_i_k_reference = nullptr)
    : PreIntegratorFactory(gyro_noise_covariance, integrator_type)
    , D_R_i_k_reference_(D_R_i_k_reference)
    , useSimpleCovariancePropagation_(useSimpleCovariancePropagation)
  {
  }

  PreIntegrator::Ptr get()
  {
    if (D_R_i_k_reference_)
    {
      return std::make_shared<ManifoldPreIntegrationState>(
            gyro_noise_covariance_,
            integrator_type_,
            useSimpleCovariancePropagation_,
            D_R_i_k_reference_);
    }
    else
    {
      return std::make_shared<ManifoldPreIntegrationState>(
            gyro_noise_covariance_,
            integrator_type_,
            useSimpleCovariancePropagation_);
    }
  }

private:
  //! The optional already pre-integrated (clean, non corrupted) orientations.
  const preintegrated_orientation_container_t* D_R_i_k_reference_;
  bool useSimpleCovariancePropagation_;
};

} // namespace ze
