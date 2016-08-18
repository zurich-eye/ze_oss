#pragma once

// The below code and structure is heavily inspired by gtsam's
// navigation/Scenario.h and ScenarioRunner.h

#include <ze/vi_simulation/trajectory_simulator.hpp>
#include <ze/vi_simulation/imu_bias_simulator.hpp>

namespace ze {

//! Given the trajectory defined by a Scenario, the runner generates
//! the corresponding corrupted and actual imu measurements.
//! The parameter naming follows the convention of a yaml-serialized
//! imu model.
class ImuSimulator
{
public:
  ZE_POINTER_TYPEDEFS(ImuSimulator);

  ImuSimulator(
      const TrajectorySimulator::Ptr& trajectory,
      const ImuBiasSimulator::Ptr& bias,
      RandomVectorSampler<3>::Ptr& accelerometer_noise,
      RandomVectorSampler<3>::Ptr& gyro_noise,
      FloatType accelerometer_noise_bandwidth_hz,
      FloatType gyroscope_noise_bandwidth_hz,
      FloatType gravity_magnitude)
    : trajectory_(trajectory)
    , bias_(bias)
    , accelerometer_noise_(accelerometer_noise)
    , gyro_noise_(gyro_noise)
    , accelerometer_noise_bandwidth_hz_sqrt_(std::sqrt(accelerometer_noise_bandwidth_hz))
    , gyro_noise_bandwidth_hz_sqrt_(std::sqrt(gyroscope_noise_bandwidth_hz))
    , gravity_(Vector3(0, 0, gravity_magnitude))
  {}

  //! The gravity vector is fixed along Z axis.
  const Vector3& gravity() const
  {
    return gravity_;
  }

  //! Get the angular velocity in the body frame.
  Vector3 angularVelocityActual(FloatType t) const
  {
    return trajectory_->angularVelocity_B(t);
  }

  //! An accelerometer measures the specific force (incl. gravity).
  Vector3 specificForceActual(FloatType t) const
  {
    Quaternion Rbw(trajectory_->R_W_B(t).inverse());
    return trajectory_->acceleration_B(t) + Rbw.rotate(gravity());
  }

  //! The angular velocity corrupted by noise and bias.
  Vector3 angularVelocityCorrupted(FloatType t) const
  {
    return angularVelocityActual(t) + bias_->gyroscope(t) +
        gyro_noise_->sample() * gyro_noise_bandwidth_hz_sqrt_;
  }

  //! The specific force corrupted by noise and bias.
  Vector3 specificForceCorrupted(FloatType t) const
  {
    return specificForceActual(t) + bias_->accelerometer(t) +
        accelerometer_noise_->sample() *
        accelerometer_noise_bandwidth_hz_sqrt_;
  }

private:
  const TrajectorySimulator::Ptr trajectory_;
  const ImuBiasSimulator::Ptr bias_;

  mutable RandomVectorSampler<3>::Ptr accelerometer_noise_;
  mutable RandomVectorSampler<3>::Ptr gyro_noise_;

  //! The noise bandwidth of accelerometer and gyroscope (sqrt(hz))
  const FloatType accelerometer_noise_bandwidth_hz_sqrt_;
  const FloatType gyro_noise_bandwidth_hz_sqrt_;

  //! The gravity vector in the world frame (negative Z-axis).
  const Vector3 gravity_;
};

} // namespace ze
