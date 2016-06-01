#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/imu/scenario.hpp>
#include <ze/imu/imu_bias.hpp>
#include <ze/common/sampler.hpp>

namespace ze {

//! Given the trajectory defined by a Scenario, the runner generates
//! the corresondping corrupted and actual imu measurements.
class ScenarioRunner
{
public:
  ScenarioRunner(const Scenario::Ptr scenario,
                 const ImuBias::Ptr bias,
                 GaussianSampler<3>::Ptr accelerometer_noise,
                 GaussianSampler<3>::Ptr gyro_noise,
                 const Vector3& gravity,
                 double imu_sample_time = 1.0 / 100.0)
      : scenario_(scenario)
      , bias_(bias)
      , accelerometer_noise_(accelerometer_noise)
      , gyro_noise_(gyro_noise)
      , gravity_(gravity)
      , imu_sample_time_(imu_sample_time)
      , sqrt_dt_(std::sqrt(imu_sample_time))
  {}

  //! The gravity vector is fixed along Z axis.
  const Vector3& gravity() const
  {
    return gravity_;
  }

  //! Get the angular velocity in the body frame.
  Vector3 angular_velocity_actual(double t) const
  {
    return scenario_->angular_velocity_B(t);
  }

  //! An accelerometer measures acceleration in the body frame, but not gravity.
  Vector3 acceleration_actual(double t) const
  {
    Quaternion Rbw(scenario_->R_W_B(t).inverse());
    return scenario_->acceleration_B(t) - Rbw.rotate(gravity());
  }

  //! The angular velocity corrupted by noise and bias.
  Vector3 angular_velocity_corrupted(double t) const
  {
    return angular_velocity_actual(t) + bias_->gyroscope(t) +
           gyro_noise_->sample() / sqrt_dt_;
  }

  //! The acceleration corrupted by noise and bias.
  Vector3 acceleration_corrupted(double t) const
  {
    return acceleration_actual(t) + bias_->accelerometer(t) +
           accelerometer_noise_->sample() / sqrt_dt_;
  }

  const double& imu_sample_time() const
  {
    return imu_sample_time_;
  }

private:
  const Scenario::Ptr scenario_;
  const ImuBias::Ptr bias_;

  mutable GaussianSampler<3>::Ptr accelerometer_noise_;
  mutable GaussianSampler<3>::Ptr gyro_noise_;

  const Vector3& gravity_;

  const FloatType imu_sample_time_;
  const FloatType sqrt_dt_;
};

} // namespace ze
