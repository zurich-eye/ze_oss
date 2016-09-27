// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
      RandomVectorSampler<3>::Ptr accelerometer_noise,
      RandomVectorSampler<3>::Ptr gyro_noise,
      real_t accelerometer_noise_bandwidth_hz,
      real_t gyroscope_noise_bandwidth_hz,
      real_t gravity_magnitude)
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
  Vector3 angularVelocityActual(real_t t) const
  {
    return trajectory_->angularVelocity_B(t);
  }

  //! An accelerometer measures the specific force (incl. gravity).
  Vector3 specificForceActual(real_t t) const
  {
    Quaternion Rbw(trajectory_->R_W_B(t).inverse());
    return trajectory_->acceleration_B(t) + Rbw.rotate(gravity());
  }

  //! The angular velocity corrupted by noise and bias.
  Vector3 angularVelocityCorrupted(real_t t) const
  {
    return angularVelocityActual(t) + bias_->gyroscope(t) +
        gyro_noise_->sample() * gyro_noise_bandwidth_hz_sqrt_;
  }

  //! The specific force corrupted by noise and bias.
  Vector3 specificForceCorrupted(real_t t) const
  {
    return specificForceActual(t) + bias_->accelerometer(t) +
        accelerometer_noise_->sample() *
        accelerometer_noise_bandwidth_hz_sqrt_;
  }

  //! Gyro and accel bias.
  const ImuBiasSimulator::Ptr& bias() const
  {
    return bias_;
  }

private:
  const TrajectorySimulator::Ptr trajectory_;
  const ImuBiasSimulator::Ptr bias_;

  mutable RandomVectorSampler<3>::Ptr accelerometer_noise_;
  mutable RandomVectorSampler<3>::Ptr gyro_noise_;

  //! The noise bandwidth of accelerometer and gyroscope (sqrt(hz))
  const real_t accelerometer_noise_bandwidth_hz_sqrt_;
  const real_t gyro_noise_bandwidth_hz_sqrt_;

  //! The gravity vector in the world frame (negative Z-axis).
  const Vector3 gravity_;
};

} // namespace ze
