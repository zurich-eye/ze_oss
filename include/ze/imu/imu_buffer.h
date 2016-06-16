#pragma once

#include <ze/imu/imu_model.h>
#include <ze/common/ringbuffer.h>

namespace ze {

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
  //! to the accelerometer, last 3 the gyroscope
  void insertImuMeasurement(int64_t time, const Vector6 value);

  //! Get the rectified values of the IMU at a given timestamp. Interpolates
  //! if necessary.
  //! Return flag if successful
  bool get(int64_t time, Eigen::Ref<Vector6> out);

  //! Get all values between two timestamps, synchronize Accelerometer and
  //! Gyroscope, interpolate edges to fit start and end. Interpolates Gyro
  //! and Accel measurements to have equal timestamps.
  std::pair<ImuStamps, ImuAccGyr>
  getBetweenValuesInterpolated(int64_t stamp_from, int64_t stamp_to);

protected:
  bool getAccelerometerDistorted(int64_t time, Eigen::Ref<Vector3> out);
  bool getGyroscopeDistorted(int64_t time, Eigen::Ref<Vector3> out);

private:
  //! The underlying storage structures for accelerometer and gyroscope
  //! measurements.
  Ringbuffer<FloatType, 3, BufferSize> acc_buffer_;
  Ringbuffer<FloatType, 3, BufferSize> gyr_buffer_;

  ImuModel::Ptr imu_model_;
};

// A set of explicit declarations
typedef ImuBuffer<2000, InterpolatorLinear> ImuBufferLinear2000;
typedef ImuBuffer<5000, InterpolatorLinear> ImuBufferLinear5000;
typedef ImuBuffer<2000, InterpolatorNearest> ImuBufferNearest2000;
typedef ImuBuffer<5000, InterpolatorNearest> ImuBufferNearest5000;

} // namespace ze
