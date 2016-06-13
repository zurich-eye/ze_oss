#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/scenario_runner.hpp>

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

  typedef std::vector<FloatType> times_container_t;
  typedef std::vector<Vector6> measurements_container_t;
  typedef std::vector<Matrix3> preintegrated_orientation_container_t;
  typedef std::vector<Matrix3> covariance_container_t;

  PreIntegrator(Matrix3 gyro_noise_covariance)
    : gyro_noise_covariance_(gyro_noise_covariance)
  {
    R_i_j_.push_back(Matrix3::Identity());
    D_R_i_j_.push_back(Matrix3::Identity());
    covariance_i_j_.push_back(Matrix3::Zero());

    R_i_k_.push_back(Matrix3::Identity());
    D_R_i_k_.push_back(Matrix3::Identity());
    covariance_i_k_.push_back(Matrix3::Zero());
  }

  //! Set the initial orientation of an absolute pre-integration term.
  void setInitialOrientation(Matrix3 initial_orientation)
  {
    R_i_j_.clear();
    R_i_k_.clear();
    R_i_j_.push_back(initial_orientation);
    R_i_k_.push_back(initial_orientation);
  }

  //! This assumes that every pushed batch corresponds to an interval between
  //! two images / keyframes.
  //! The input measurements should be bias corrected.
  virtual void pushD_R_i_j(times_container_t imu_stamps,
                       measurements_container_t imu_measurements) = 0;

  //! Get the result of the pre-integration process.
  preintegrated_orientation_container_t D_R_i_j()
  {
    return D_R_i_j_;
  }

  //! Get the absolute orientation after pre-integration.
  preintegrated_orientation_container_t R_i_j()
  {
    return R_i_j_;
  }

  //! Get the timestamps corresponding to the pre-integrated orientations,
  //! Where the i'th element in the orientation container refers to the
  //! time interval: [i, i+1].
  times_container_t times()
  {
    return times_;
  }

  //! Get the propagated covariance matrix.
  covariance_container_t covariance_i_j()
  {
    return covariance_i_j_;
  }

  // Getters for the above quantities at the sampling rate of the imu.
  preintegrated_orientation_container_t D_R_i_k()
  {
    return D_R_i_k_;
  }

  preintegrated_orientation_container_t R_i_k()
  {
    return R_i_k_;
  }

  times_container_t times_raw()
  {
    return times_raw_;
  }

  covariance_container_t covariance_i_k()
  {
    return covariance_i_k_;
  }

  const measurements_container_t& measurements()
  {
    return measurements_;
  }

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
};

} // namespace ze
