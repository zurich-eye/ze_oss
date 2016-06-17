#include <ze/imu_evaluation/imu_bias.hpp>

namespace ze {

//------------------------------------------------------------------------------
ContinuousBias::ContinuousBias(const Vector3& gyr_bias_noise_density,
               const Vector3 acc_bias_noise_density,
               FloatType start,
               FloatType end,
               size_t samples,
               size_t spline_order,
               size_t spline_segments,
               size_t spline_smoothing_lambda)
  : gyr_bias_noise_density_(gyr_bias_noise_density)
  , acc_bias_noise_density_(acc_bias_noise_density)
  , start_(start)
  , end_(end)
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
void ContinuousBias::initialize()
{
  Vector6 noise;
  noise.head<3>() = acc_bias_noise_density_;
  noise.tail<3>() = gyr_bias_noise_density_;
  // merge acc and bias noise
  RandomVectorSampler<6>::Ptr sampler = RandomVectorSampler<6>::sigmas(noise);

  // sampling interval
  FloatType dt = (end_ - start_) / samples_;
  FloatType dt_sqrt = sqrt(dt);
  CHECK_LE(0, dt);

  // simulate the white noise process
  MatrixX points(6, samples_ + 1);
  VectorX times(samples_ + 1);
  points.block<6, 1>(0, 0);
  times(0) = start_;
  for (size_t i = 1; i <= samples_; ++i)
  {
    times(i) = start_ + dt * i;
    points.block<6, 1>(0, i) = points.block<6, 1>(0, i - 1) +
        dt_sqrt * sampler->sample();
  }

  // initialize spline
  bs_.initSpline3(times, points, spline_segments_, spline_smoothing_lambda_);
}

} // namespace ze
