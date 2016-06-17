#include <ze/imu/imu_buffer.h>

namespace ze {

template<int BufferSize, typename Interpolator>
ImuBuffer<BufferSize, Interpolator>::ImuBuffer(ImuModel::Ptr imu_model)
  : imu_model_(imu_model)
  , gyro_delay_(secToNanosec(imu_model->gyroscopeModel()->intrinsicModel()->delay()))
  , accel_delay_(secToNanosec(imu_model->accelerometerModel()->intrinsicModel()->delay()))
{
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertImuMeasurement(
    int64_t time, const ImuAccGyr value)
{
  acc_buffer_.insert(correctStampAccel(time), value.head<3>(3));
  gyr_buffer_.insert(correctStampGyro(time), value.tail<3>(3));
}


template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertGyroscopeMeasurement(
    int64_t time, const Vector3 value)
{
  gyr_buffer_.insert(correctStampGyro(time), value);
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertAccelerometerMeasurement(
    int64_t time, const Vector3 value)
{
  acc_buffer_.insert(correctStampAccel(time), value);
}

template<int BufferSize, typename Interpolator>
bool ImuBuffer<BufferSize, Interpolator>::get(int64_t time,
                                              Eigen::Ref<ImuAccGyr> out)
{
  if (!acc_buffer_.getValueInterpolated(time, out.head<3>(3)) ||
      !gyr_buffer_.getValueInterpolated(time, out.tail<3>(3)))
  {
    return false;
  }

  imu_model_->undistort(out);

  return true;
}

template<int BufferSize, typename Interpolator>
bool ImuBuffer<BufferSize, Interpolator>::getAccelerometerDistorted(
    int64_t time,
    Eigen::Ref<Vector3> out)
{
  return acc_buffer_.getValueInterpolated(time, out);
}

template<int BufferSize, typename Interpolator>
bool ImuBuffer<BufferSize, Interpolator>::getGyroscopeDistorted(
    int64_t time,
    Eigen::Ref<Vector3> out)
{
  return gyr_buffer_.getValueInterpolated(time, out);
}

template<int BufferSize, class Interpolator>
std::pair<ImuStamps, ImuAccGyrContainer>
ImuBuffer<BufferSize, Interpolator>::getBetweenValuesInterpolated(
    int64_t stamp_from, int64_t stamp_to)
{
  // Takes the gyroscope time as reference and samples the accelerometer
  // measurements to fit
  ImuStamps stamps;
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> gyr_measurements;

  std::tie(stamps, gyr_measurements) =
      gyr_buffer_.template getBetweenValuesInterpolated<Interpolator>(
        stamp_from, stamp_to);

  ImuAccGyrContainer imu_measurements(6, stamps.size());
  if (stamps.size() == 0)
  {
    // return an empty set:
    return std::make_pair(stamps, imu_measurements);
  }

  // resample the accelerometer measurement at the gyro timestamps
  imu_measurements.topRows<3>() = acc_buffer_.getValuesInterpolated(stamps);
  imu_measurements.bottomRows<3>() = gyr_measurements;

  return std::make_pair(stamps, imu_measurements);
}

template<int BufferSize, class Interpolator>
std::tuple<int64_t, int64_t, bool>
ImuBuffer<BufferSize, Interpolator>::getOldestAndNewestStamp() const
{
  std::tuple<int64_t, int64_t, bool> accel =
      acc_buffer_.getOldestAndNewestStamp();
  std::tuple<int64_t, int64_t, bool> gyro =
      gyr_buffer_.getOldestAndNewestStamp();

  if (!std::get<2>(accel) || !std::get<2>(gyro))
  {
    return std::make_tuple(-1, -1, false);
  }

  int64_t oldest = std::get<0>(accel) < std::get<0>(gyro) ?
                     std::get<0>(gyro) : std::get<0>(accel);

  int64_t newest = std::get<1>(accel) < std::get<1>(gyro) ?
                     std::get<1>(accel) : std::get<1>(gyro);

  // This is an extreme edge case where the accel and gyro measurements
  // do not overlap at all.
  if (oldest > newest)
  {
    return std::make_tuple(-1, -1, false);
  }

  return std::make_tuple(oldest, newest, true);
}

// A set of explicit declarations
template class ImuBuffer<2000, InterpolatorLinear>;
template class ImuBuffer<5000, InterpolatorLinear>;
template class ImuBuffer<2000, InterpolatorNearest>;
template class ImuBuffer<5000, InterpolatorNearest>;

} // namespace ze
