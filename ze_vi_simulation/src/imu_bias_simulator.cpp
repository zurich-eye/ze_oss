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

#include <ze/vi_simulation/imu_bias_simulator.hpp>

namespace ze {

//------------------------------------------------------------------------------
ContinuousBiasSimulator::ContinuousBiasSimulator(
    const Vector3& gyr_bias_noise_density,
    const Vector3& acc_bias_noise_density,
    real_t start_time,
    real_t end_time,
    size_t samples,
    size_t spline_order,
    size_t spline_segments,
    size_t spline_smoothing_lambda)
  : gyr_bias_noise_density_(gyr_bias_noise_density)
  , acc_bias_noise_density_(acc_bias_noise_density)
  , start_(start_time)
  , end_(end_time)
  , samples_(samples)
  , spline_order_(spline_order)
  , spline_segments_(spline_segments)
  , spline_smoothing_lambda_(spline_smoothing_lambda)
  , bs_(3)
{
  if (spline_segments_ == 0)
  {
    // this is usually a good setting
    spline_segments_ = samples / 2;
  }
  initialize();
}

//------------------------------------------------------------------------------
void ContinuousBiasSimulator::initialize()
{
  Vector6 noise;
  noise.head<3>() = acc_bias_noise_density_;
  noise.tail<3>() = gyr_bias_noise_density_;
  // merge acc and bias noise
  RandomVectorSampler<6>::Ptr sampler = RandomVectorSampler<6>::sigmas(noise);

  // sampling interval
  real_t dt = (end_ - start_) / samples_;
  real_t dt_sqrt = sqrt(dt);
  CHECK_LE(0, dt);

  // simulate the white noise process
  MatrixX points(6, samples_ + 1);
  VectorX times(samples_ + 1);
  points.col(0) = Vector6::Zero();
  times(0) = start_;
  for (size_t i = 1; i <= samples_; ++i)
  {
    times(i) = start_ + dt * i;
    points.col(i) = points.col(i-1) +
        dt_sqrt * sampler->sample();
  }

  // initialize spline
  bs_.initSpline3(times, points, spline_segments_, spline_smoothing_lambda_);
}

} // namespace ze
