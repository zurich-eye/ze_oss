// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/timer_collection.hpp>
#include <ze/vi_simulation/imu_simulator.hpp>

namespace ze {

//! The base-class for all pre-integrators.
//! The container re-uses the notation of:
//! On-Manifold Preintegration for Real-time Visual-Inertial Odometry (Forster et al.):
//! i: index of the previous frame;
//! j: index of the current frame;
//! k: index of an imu measurement between i and j.
class PreIntegrator
{
public:
   ZE_POINTER_TYPEDEFS(PreIntegrator);

   enum IntegratorType
   {
     FirstOrderForward,
     FirstOrderMidward,
     RungeKutta3,
     RungeKutta4,
     CrouchGrossman3,
     CrouchGrossman4
   };

  typedef std::vector<real_t> times_container_t;
  typedef ImuAccGyrContainer measurements_container_t;
  typedef std::vector<Matrix3> preintegrated_orientation_container_t;
  typedef std::vector<Matrix3> covariance_container_t;

  PreIntegrator(Matrix3 gyro_noise_covariance, IntegratorType integrator_type);

  //! Compute absolute values or not.
  void computeAbsolutes(bool flag)
  {
    compute_absolutes_ = flag;
  }

  //! Set the initial orientation of an absolute pre-integration term.
  virtual void setInitialOrientation(Matrix3 initial_orientation);

  void pushD_R_i_j(times_container_t imu_stamps,
                   measurements_container_t imu_measurements);
  //! This assumes that every pushed batch corresponds to an interval between
  //! two images / keyframes.
  //! The input measurements should be bias corrected.
  virtual void integrate(times_container_t imu_stamps,
                             measurements_container_t imu_measurements) = 0;

  //! Get the result of the pre-integration process.
  const preintegrated_orientation_container_t& D_R_i_j() const
  {
    return D_R_i_j_;
  }

  //! Get the absolute orientation after pre-integration.
  const preintegrated_orientation_container_t& R_i_j() const
  {
    return R_i_j_;
  }

  //! Get the timestamps corresponding to the pre-integrated orientations,
  //! Where the i'th element in the orientation container refers to the
  //! time interval: [i, i+1].
  const times_container_t& times() const  {
    return times_;
  }

  //! Get the propagated covariance matrix.
  const covariance_container_t& covariance_i_j() const
  {
    return covariance_i_j_;
  }

  // Getters for the above quantities at the sampling rate of the imu.
  const preintegrated_orientation_container_t& D_R_i_k() const
  {
    return D_R_i_k_;
  }

  const preintegrated_orientation_container_t& R_i_k() const
  {
    return R_i_k_;
  }

  const times_container_t& times_raw() const
  {
    return times_raw_;
  }

  const covariance_container_t& covariance_i_k() const
  {
    return covariance_i_k_;
  }

  const measurements_container_t& measurements() const
  {
    return measurements_;
  }

  //! A collection of timers.
  DECLARE_TIMER(IntegrationTimer, timers_, integrate);

protected:
  //! The time at which a frame was captured.
  times_container_t times_;

  //! The timestamp of every imu measurement in the container.
  times_container_t times_raw_;

  // The pre-integrated values given the keyframe based steps.
  //! The relative rotation at a given image frame.
  preintegrated_orientation_container_t D_R_i_j_;

  //! The absolute rotation at a given image frame.
  preintegrated_orientation_container_t R_i_j_;

  //! The covariances at the pre-integration steps.
  covariance_container_t covariance_i_j_;

  // The raw pre-integrated values at every single imu-measurement.
  preintegrated_orientation_container_t D_R_i_k_;
  preintegrated_orientation_container_t R_i_k_;
  covariance_container_t covariance_i_k_;

  // Keep track of all incoming measurements.
  measurements_container_t measurements_;

  //! The covariance matrix of the gyroscope noise.
  Matrix3 gyro_noise_covariance_;

  //! The type of integration (fwd, bwd, midpoint etc.)
  IntegratorType integrator_type_;

  //! Should the preintegrator estimate the absolute values or only relatives?
  bool compute_absolutes_;
};

//------------------------------------------------------------------------------
//! A base class for pre-integrator factory objects
class PreIntegratorFactory
{
public:
  ZE_POINTER_TYPEDEFS(PreIntegratorFactory);

  PreIntegratorFactory(Matrix3 gyro_noise_covariance,
                       PreIntegrator::IntegratorType integrator_type)
    : gyro_noise_covariance_(gyro_noise_covariance)
    , integrator_type_(integrator_type)
  {
  }

  //! Get an instance of the Pre-Integrator.
  virtual PreIntegrator::Ptr get() = 0;

protected:
  //! The covariance matrix of the gyroscope noise.
  Matrix3 gyro_noise_covariance_;

  //! The type of integration (fwd, bwd, midpoint etc.)
  PreIntegrator::IntegratorType integrator_type_;
};

} // namespace ze
