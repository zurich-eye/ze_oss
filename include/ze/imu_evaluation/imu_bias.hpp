#pragma once

#include <ze/common/types.h>
#include <ze/common/random_matrix.hpp>
#include <ze/splines/bspline.hpp>

namespace ze {

//! The base class for Imu Bias simulations.
class ImuBias
{
public:
  ZE_POINTER_TYPEDEFS(ImuBias);

  //! Access the accelerometer bias at given timestamp.
  virtual const Vector3 accelerometer(FloatType t) const = 0;
  //! Access the gyroscope bias at given timestamp.
  virtual const Vector3 gyroscope(FloatType t) const = 0;

  //! Regenerate the bias .
  virtual void reset() = 0;
};

//! A simple constant bias defined at all timestamps.
class ConstantBias : public ImuBias
{
public:
  ConstantBias()
    : bias_acc_(0.0, 0.0, 0.0)
    , bias_gyro_(0.0, 0.0, 0.0)
  {
  }

  ConstantBias(const Vector3& bias_acc, const Vector3& bias_gyro) :
      bias_acc_(bias_acc), bias_gyro_(bias_gyro) {
  }

  const Vector3 accelerometer(FloatType t) const
  {
    return bias_acc_;
  }

  const Vector3 gyroscope(FloatType t) const
  {
    return bias_gyro_;
  }

  void reset()
  {
  }

private:
  Vector3 bias_acc_;
  Vector3 bias_gyro_;
};

//! A continous-time bias curve seeded from a discrete random walk.
class ContinuousBias : public ImuBias
{
public:
  //! Given the process noise, start/end times and number of samples to take
  //! initializes a spline from a discrete random walk.
  ContinuousBias(const Vector3& gyr_bias_noise_density,
                 const Vector3 acc_bias_noise_density,
                 FloatType start,
                 FloatType end,
                 size_t samples,
                 size_t spline_order = 3,
                 size_t spline_segments = 0,
                 size_t spline_smoothing_lambda = 1e-5);

  const Vector3 accelerometer(FloatType t) const
  {
    CHECK_GE(t, start_);
    CHECK_LE(t, end_);
    return bs_.eval(t).head<3>();
  }

  const Vector3 gyroscope(FloatType t) const
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
  FloatType start_;
  FloatType end_;

  size_t samples_;
  size_t spline_order_;
  size_t spline_segments_;
  FloatType spline_smoothing_lambda_;

  //! The first three elements are the accelerometer bias, last 3 elements are
  //! the gyrocope bias.
  BSpline bs_;

};

} // namespace ze
