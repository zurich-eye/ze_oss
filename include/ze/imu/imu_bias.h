#pragma once

#include <ze/common/types.h>
#include <ze/common/sampler.h>
#include <ze/splines/bspline.h>

namespace ze {

//! base class for Imu Bias simulations
class ImuBias
{
  //! access accelerometer bias at given timestamp
  virtual const Vector3 accelerometer(FloatType t) const = 0;
  //! access gyroscope bias at given timestamp
  virtual const Vector3 gyroscope(FloatType t) const = 0;

  //! reset/regenerate bias
  virtual void reset() = 0;
};

//! simple constant bias assumption
//! time unconstrained
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

//! random walk bias with spline fitting for continuity
class ContinuousBias : public ImuBias
{
public:
  //! given process noise and start/end times and number of samples to take
  ContinuousBias(const Vector3& gyr_bias_noise,
                 const Vector3 acc_bias_noise,
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

  // others: pass sampler to scenario for noise sampling;
  // pass imu bias to sampler and properly use
  // whats next?

  Vector3 gyr_bias_noise_;
  Vector3 acc_bias_noise_;
  FloatType start_;
  FloatType end_;
  size_t samples_;
  size_t spline_order_;
  size_t spline_segments_;
  FloatType spline_smoothing_lambda_;

  //! head<3>: accelerometer, tail<3> gyroscope
  BSpline bs_;

};

} // namespace ze
