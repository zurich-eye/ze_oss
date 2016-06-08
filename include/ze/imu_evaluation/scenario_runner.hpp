#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/imu_evaluation/scenario.hpp>
#include <ze/imu_evaluation/imu_bias.hpp>
#include <ze/common/sampler.hpp>

namespace ze {

//! Given the trajectory defined by a Scenario, the runner generates
//! the corresondping corrupted and actual imu measurements.
//! The parameter naming follows the convention of a yaml-serialized
//! imu model.
class ScenarioRunner
{
public:
  //!
  ScenarioRunner(const Scenario::Ptr scenario,
                 const ImuBias::Ptr bias,
                 RandomVectorSampler<3>::Ptr accelerometer_noise,
                 RandomVectorSampler<3>::Ptr gyro_noise,
                 FloatType accelerometer_noise_bandwidth_hz,
                 FloatType gyroscope_noise_bandwidth_hz,
                 const Vector3& gravity)
    : scenario_(scenario)
    , bias_(bias)
    , accelerometer_noise_(accelerometer_noise)
    , gyro_noise_(gyro_noise)
    , accelerometer_noise_bandwidth_hz_sqrt_(sqrt(accelerometer_noise_bandwidth_hz))
    , gyro_noise_bandwidth_hz_sqrt_(sqrt(gyroscope_noise_bandwidth_hz))
    , gravity_(gravity)
  {}

  //! The gravity vector is fixed along Z axis.
  const Vector3& gravity() const
  {
    return gravity_;
  }

  //! Get the angular velocity in the body frame.
  Vector3 angular_velocity_actual(FloatType t) const
  {
    return scenario_->angular_velocity_B(t);
  }

  //! An accelerometer measures the specific force (incl. gravity).
  Vector3 specific_force_actual(FloatType t) const
  {
    Quaternion Rbw(scenario_->R_W_B(t).inverse());
    return scenario_->acceleration_B(t) + Rbw.rotate(gravity());
  }

  //! The angular velocity corrupted by noise and bias.
  Vector3 angular_velocity_corrupted(FloatType t) const
  {
    return angular_velocity_actual(t) + bias_->gyroscope(t) +
        gyro_noise_->sample() * gyro_noise_bandwidth_hz_sqrt_;
  }

  //! The specific force corrupted by noise and bias.
  Vector3 specific_force_corrupted(FloatType t) const
  {
    return specific_force_actual(t) + bias_->accelerometer(t) +
        accelerometer_noise_->sample() *
        accelerometer_noise_bandwidth_hz_sqrt_;
  }

private:
  const Scenario::Ptr scenario_;
  const ImuBias::Ptr bias_;

  mutable RandomVectorSampler<3>::Ptr accelerometer_noise_;
  mutable RandomVectorSampler<3>::Ptr gyro_noise_;

  //! The noise bandwidth of accelerometer and gyroscope (sqrt(hz))
  const FloatType accelerometer_noise_bandwidth_hz_sqrt_;
  const FloatType gyro_noise_bandwidth_hz_sqrt_;

  //! The gravity vector in the world frame (negative Z-axis).
  const Vector3& gravity_;
};

} // namespace ze
