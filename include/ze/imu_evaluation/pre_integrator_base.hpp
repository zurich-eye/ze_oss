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
    R_i_.push_back(Matrix3::Identity());
    D_R_.push_back(Matrix3::Identity());
    covariances_.push_back(Matrix3::Zero());
  }

  //! This assumes that every pushed batch corresponds to an interval between
  //! two images / keyframes.
  //! The input measurements should be bias corrected.
  virtual void pushD_R(times_container_t imu_stamps,
                       measurements_container_t imu_measurements) = 0;

  //! Calculate the linearly propagated covariance for the last segment.
  virtual void propagate_covariance() = 0;

  //! Get the result of the pre-integration process.
  preintegrated_orientation_container_t getD_R()
  {
    return D_R_;
  }

  //! Get the absolute orientation after pre-integration.
  preintegrated_orientation_container_t getR_i()
  {
    return R_i_;
  }

  //! Get the timestamps corresponding to the pre-integrated orientations,
  //! Where the i'th element in the orientation container refers to the
  //! time interval: [i, i+1].
  times_container_t getTimes()
  {
    return times_;
  }

  //! Get the propagated covariance matrix.
  covariance_container_t getCovariance()
  {
    return covariances_;
  }

protected:
  //! The time at which a frame was captured.
  times_container_t times_;

  //! The relative rotation at a given image frame.
  preintegrated_orientation_container_t D_R_;

  //! The absolute rotation at a given image frame.
  preintegrated_orientation_container_t R_i_;

  //! The covariances at the pre-integration steps.
  covariance_container_t covariances_;

  //! The covariance matrix of the gyroscope noise.
  Matrix3 gyro_noise_covariance_;
};

} // namespace ze
