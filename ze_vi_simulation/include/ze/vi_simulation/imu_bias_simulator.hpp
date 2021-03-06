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

#include <ze/common/types.hpp>
#include <ze/common/random_matrix.hpp>
#include <ze/splines/bspline.hpp>

namespace ze {

//! The base class for Imu Bias simulations.
class ImuBiasSimulator
{
public:
  ZE_POINTER_TYPEDEFS(ImuBiasSimulator);

  //! Access the accelerometer bias at given timestamp.
  virtual const Vector3 accelerometer(real_t t) const = 0;

  //! Access the gyroscope bias at given timestamp.
  virtual const Vector3 gyroscope(real_t t) const = 0;

  //! Regenerate the bias.
  virtual void reset() = 0;
};

//! A simple constant bias defined at all timestamps.
class ConstantBiasSimulator : public ImuBiasSimulator
{
public:
  ConstantBiasSimulator() = default;

  ConstantBiasSimulator(const Vector3& bias_acc, const Vector3& bias_gyro) :
      bias_acc_(bias_acc), bias_gyro_(bias_gyro) {
  }

  //! Get accelerometer bias at time t.
  const Vector3 accelerometer(real_t t) const override
  {
    return bias_acc_;
  }

  //! Get gyroscope bias at time t.
  const Vector3 gyroscope(real_t t) const override
  {
    return bias_gyro_;
  }

  void reset() override
  {}

private:
  Vector3 bias_acc_ = Vector3::Zero();
  Vector3 bias_gyro_ = Vector3::Zero();
};

//! A continous-time bias curve seeded from a discrete random walk.
class ContinuousBiasSimulator : public ImuBiasSimulator
{
public:
  //! Given the process noise, start/end times and number of samples to take
  //! initializes a spline from a discrete random walk.
  ContinuousBiasSimulator(
      const Vector3& gyr_bias_noise_density,
      const Vector3& acc_bias_noise_density,
      real_t start_time,
      real_t end_time,
      size_t samples,
      size_t spline_order = 3,
      size_t spline_segments = 0,
      size_t spline_smoothing_lambda = 1e-5);

  //! Get accelerometer bias at time t.
  const Vector3 accelerometer(real_t t) const
  {
    CHECK_GE(t, start_);
    CHECK_LE(t, end_);
    return bs_.eval(t).head<3>();
  }

  //! Get gyroscope bias at time t.
  const Vector3 gyroscope(real_t t) const
  {
    CHECK_GE(t, start_);
    CHECK_LE(t, end_);
    return bs_.eval(t).tail<3>();
  }

  void reset()
  {
    initialize();
  }

private:
  void initialize();

  //! The noise density of the gyro / accelerometer brownian motion.
  Vector3 gyr_bias_noise_density_;
  Vector3 acc_bias_noise_density_;

  //! Start and End-Time of the modelled continous bias.
  real_t start_;
  real_t end_;

  size_t samples_;
  size_t spline_order_;
  size_t spline_segments_;
  real_t spline_smoothing_lambda_;

  //! The first three elements are the accelerometer bias, last 3 elements are
  //! the gyrocope bias.
  BSpline bs_;

};

} // namespace ze
