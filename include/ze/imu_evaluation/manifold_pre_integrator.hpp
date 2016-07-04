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
  //! Set useSimpleCovariancePropagation to skip the Jacobian calculation for
  //! noise propagation and use Identity.
  ManifoldPreIntegrationState(
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type,
      bool useSimpleCovariancePropagation = false,
      const preintegrated_orientation_container_t* D_R_i_k_reference = nullptr);

  void integrate(times_container_t stamps,
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

  void integrateFirstOrderFwd(times_container_t stamps,
                           measurements_container_t measurements);

  void integrateFirstOrderMid(times_container_t stamps,
                           measurements_container_t measurements);

  //! Set the values used for a new pre-integration batch.
  void setInitialValuesFwd(const FloatType dt,
                            const Eigen::Ref<Vector3>& gyro_measurement);

  //! Set the values used for a new pre-integration batch.
  inline void setInitialValuesMid(const FloatType dt,
                            const Eigen::Ref<Vector3>& gyro_measurement,
                            const Eigen::Ref<Vector3>& gyro_measurement2);

  //! Push a new integration step to the containers.
  inline void integrateStepFwd(const FloatType dt,
                                 const Eigen::Ref<Vector3>& gyro_measurement);

  //! Push a new integration step to the containers.
  void integrateStepMid(
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
