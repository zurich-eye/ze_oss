// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/pre_integrator_base.hpp>
#include <ze/common/transformation.h>
#include <ze/vi_simulation/imu_simulator.hpp>

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
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type);

  ~QuaternionPreIntegrationState() {}

  void setInitialOrientation(Matrix3 initial_orientation) override;
  void integrate(times_container_t stamps,
                     measurements_container_t measurements);
private:
  preintegrated_orientation_quat_container_t D_R_i_j_quat_;
  preintegrated_orientation_quat_container_t R_i_j_quat_;
  preintegrated_orientation_quat_container_t D_R_i_k_quat_;
  preintegrated_orientation_quat_container_t R_i_k_quat_;

  //! Forward integration.
  inline Quaternion integrateFirstOrderFwdStep(Quaternion q,
                                    Vector3 w_i,
                                    real_t dt);

  //! Midpoint integration.
  inline Quaternion integrateFirstOrderMidStep(Quaternion q,
                                    Vector3 w_i,
                                    Vector3 w_i_1,
                                    real_t dt);

  //! Runge-Kutta Integration
  //! Returns the quaternion after preintegration and
  //! the state propagation matrix to propagate the
  //! covariance.
  inline std::pair<Quaternion, Matrix3> integrateRK(Quaternion q,
                                             Vector3 w_i,
                                             Vector3 w_i_1,
                                             real_t dt,
                                             uint32_t order);

  //! Runge-Kutta Integration
  inline Quaternion integrateCrouchGrossman(Quaternion q,
                         Vector3 w_i,
                         Vector3 w_i_1,
                         real_t dt,
                         uint32_t order);

  void integrateFirstOrderFwd(times_container_t stamps,
                           measurements_container_t measurements);
  void integrateFirstOrderMid(times_container_t stamps,
                           measurements_container_t measurements);
  void integrateRK(times_container_t stamps,
                 measurements_container_t measurements,
                uint32_t order = 3);
  void integrateCrouchGrossman(times_container_t stamps,
                            measurements_container_t measurements,
                            uint32_t order = 3);

};

//-------------------------------------------------------------------------------
//! The factory for quaternion preintegrators
class QuaternionPreIntegrationFactory: public PreIntegratorFactory
{
public:
  typedef QuaternionPreIntegrationState::preintegrated_orientation_container_t preintegrated_orientation_container_t;

  //! Optionally provide the reference (non-corrupted) pre-integrated orientation
  //! to inject into the created manifold preintegrators.
  QuaternionPreIntegrationFactory(
      Matrix3 gyro_noise_covariance,
      PreIntegrator::IntegratorType integrator_type)
    : PreIntegratorFactory(gyro_noise_covariance, integrator_type)
  {
  }

  PreIntegrator::Ptr get()
  {
    return std::make_shared<QuaternionPreIntegrationState>(gyro_noise_covariance_,
                                                           integrator_type_);
  }
};

} // namespace ze
