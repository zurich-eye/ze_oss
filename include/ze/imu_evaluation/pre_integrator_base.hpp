#pragma once

#include <ze/common/types.h>
#include <ze/imu_evaluation/scenario_runner.hpp>

namespace ze {

//! The base-class for all pre-integrators
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
    covariances_.push_back(Matrix3::Zero());
  }

  //! This assumes that every pushed batch corresponds to an interval between
  //! two images / keyframes.
  //! The input measurements should be bias corrected.
  virtual void pushD_R_i_j(times_container_t imu_stamps,
                       measurements_container_t imu_measurements) = 0;

  //! Calculate the linearly propagated covariance for the last segment.
  virtual void propagateCovariance() = 0;

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
  covariance_container_t covariances()
  {
    return covariances_;
  }

protected:
  //! The time at which a frame was captured.
  times_container_t times_;

  //! The relative rotation at a given image frame.
  preintegrated_orientation_container_t D_R_i_j_;

  //! The absolute rotation at a given image frame.
  preintegrated_orientation_container_t R_i_j_;

  //! The covariances at the pre-integration steps.
  covariance_container_t covariances_;

  //! The covariance matrix of the gyroscope noise.
  Matrix3 gyro_noise_covariance_;
};

} // namespace ze
