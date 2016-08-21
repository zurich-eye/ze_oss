#pragma once

#include <mutex>

#include <ze/imu/imu_model.h>
#include <ze/common/ringbuffer.h>

namespace ze {

struct InterpolatorDifferentiatorLinear
{
  typedef Vector6 return_t;

  //! The name could be more descriptive, but the current naming allows for
  //! using the interpolators defined in ringbuffer.h
  template<typename Ringbuffer_T>
  static return_t interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      const typename Ringbuffer_T::timering_t::iterator it_before)
  {
    // the end value
    const auto it_after = it_before + 1;
    if (it_after == buffer->times().end())
    {
      LOG(WARNING) << "Interpolation hit end of buffer.";
      return (return_t() << buffer->data().col(it_before.container_index()),
          Vector3::Zero()).finished();
    }

    const real_t offset = static_cast<FloatType>(time - *it_before);
    const real_t duration = static_cast<FloatType>(*it_after - *it_before);
    const Vector3 before = buffer->data().col(it_before.container_index());
    const Vector3 after = buffer->data().col(it_after.container_index());

    return (return_t() << before + (after - before) * offset / duration,
        (after - before) / duration).finished();
  }

  template<typename Ringbuffer_T>
  static return_t interpolate(
      Ringbuffer_T* buffer,
      int64_t time)
  {
    const auto it_before = buffer->iterator_equal_or_before(time);
    // caller should check the bounds:
    CHECK(it_before != buffer->times().end());

    return interpolate(buffer, time, it_before);
  }
};
using DefaultInterpolator = InterpolatorLinear;

//! An IMU Buffer with an underlying Gyro and Accel model that also corrects
//! measurement timestamps. The timestamps are corrected when inserted into the
//! buffers.
template<int BufferSize, typename GyroInterp,
typename AccelInterp = GyroInterp>
class ImuBuffer
{
public:
  ZE_POINTER_TYPEDEFS(ImuBuffer);

  ImuBuffer(ImuModel::Ptr imu_model);

  void insertGyroscopeMeasurement(time_t stamp, const Vector3);
  void insertAccelerometerMeasurement(time_t stamp, const Vector3);

  //! Insert an IMU measurement at a given timestamp: First three values refer
  //! to the accelerometer, last 3 the gyroscope.
  void insertImuMeasurement(int64_t time, const ImuAccGyr value);

  //! Get the rectified values of the IMU at a given timestamp. Interpolates
  //! if necessary.
  //! Return flag if successful
  bool get(int64_t time, Eigen::Ref<ImuAccGyr> out);

  //! Get all values between two timestamps, synchronize Accelerometer and
  //! Gyroscope, interpolate edges to fit start and end. Interpolates Gyro
  //! and Accel measurements to have equal timestamps. Rectify all measurements.
  std::pair<ImuStamps, ImuAccGyrContainer>
  getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to);

  //! Get the oldest and newest timestamps for which both Accelerometers
  //! and Gyroscopes have measurements.
  std::tuple<int64_t, int64_t, bool> getOldestAndNewestStamp() const;

  //! Get the delay corrected timestamps (Delays are negative if in the past).
  inline int64_t correctStampGyro(int64_t t)
  {
    return t + gyro_delay_;
  }
  inline int64_t correctStampAccel(int64_t t)
  {
     return t + accel_delay_;
  }

protected:
  bool getAccelerometerDistorted(int64_t time, Eigen::Ref<Vector3> out);
  bool getGyroscopeDistorted(int64_t time, Eigen::Ref<Vector3> out);

private:
  //! The underlying storage structures for accelerometer and gyroscope
  //! measurements.
  Ringbuffer<real_t, 3, BufferSize> acc_buffer_;
  Ringbuffer<real_t, 3, BufferSize> gyr_buffer_;

  ImuModel::Ptr imu_model_;

  //! Store the accelerometer and gyroscope delays in nanoseconds
  int64_t gyro_delay_;
  int64_t accel_delay_;
};

// A set of explicit declarations
typedef ImuBuffer<2000, InterpolatorLinear> ImuBufferLinear2000;
typedef ImuBuffer<5000, InterpolatorLinear> ImuBufferLinear5000;
typedef ImuBuffer<2000, InterpolatorNearest> ImuBufferNearest2000;
typedef ImuBuffer<5000, InterpolatorNearest> ImuBufferNearest5000;
typedef ImuBuffer<2000, InterpolatorDifferentiatorLinear, InterpolatorLinear>
ImuBufferDiff2000;
typedef ImuBuffer<5000, InterpolatorDifferentiatorLinear, InterpolatorLinear>
ImuBufferDiff5000;

} // namespace ze
