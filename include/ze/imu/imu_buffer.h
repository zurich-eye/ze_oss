#pragma once

#include <ze/imu/imu_model.h>
#include <ze/common/ringbuffer.h>

namespace ze {

//! An IMU Buffer with an underlying Gyro and Accel model that also corrects
//! measurement timestamps. The timestamps are corrected when inserted into the
//! buffers.
template<int BufferSize, class Interpolator>
class ImuBuffer
{
public:
  ZE_POINTER_TYPEDEFS(ImuBuffer);

  ImuBuffer(ImuModel::Ptr imu_model);

  using BufferInterpolator = Interpolator;

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
  //! and Accel measurements to have equal timestamps.
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
  Ringbuffer<FloatType, 3, BufferSize> acc_buffer_;
  Ringbuffer<FloatType, 3, BufferSize> gyr_buffer_;

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

} // namespace ze
