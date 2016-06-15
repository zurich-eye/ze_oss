#include <ze/imu/imu_buffer.h>

namespace ze {

template<int BufferSize, typename Interpolator>
ImuBuffer<BufferSize, Interpolator>::ImuBuffer(ImuModel::Ptr imu_model)
  : imu_model_(imu_model)
{
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertImuMeasurement(
    int64_t time, const Vector6 value)
{
  acc_buffer_.insert(time, value.head<3>(3));
  gyr_buffer_.insert(time, value.tail<3>(3));
}


template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertGyroscopeMeasurement(
    int64_t time, const Vector3 value)
{
  gyr_buffer_.insert(time, value);
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertAccelerometerMeasurement(
    int64_t time, const Vector3 value)
{
  acc_buffer_.insert(time, value);
}

template<int BufferSize, typename Interpolator>
Vector6 ImuBuffer<BufferSize, Interpolator>::get(int64_t time)
{
  Vector6 out;
  out.head<3>(3) = Interpolator::interpolate(acc_buffer_, time);
  out.tail<3>(3) = Interpolator::interpolate(gyr_buffer_, time);

  return imu_model_->undistort(out);
}

template<int BufferSize, typename Interpolator>
Vector3 ImuBuffer<BufferSize, Interpolator>::getAccelerometer(int64_t time)
{
  return Interpolator::interpolate(acc_buffer_, time);
}

template<int BufferSize, typename Interpolator>
Vector3 ImuBuffer<BufferSize, Interpolator>::getGyroscope(int64_t time)
{
  return Interpolator::interpolate(gyr_buffer_, time);
}

template<int BufferSize, typename Interpolator>
std::pair<ImuStamps, ImuAccGyr>
ImuBuffer<BufferSize, Interpolator>::getBetweenValuesInterpolated(
    int64_t stamp_from, int64_t stamp_to)
{
  // Takes the gyroscope time as reference and samples the accelerometer
  // measurements to fit
  ImuStamps stamps;
  ImuAccGyr imu_measurements;
  Eigen::Matrix<FloatType, 3, Eigen::Dynamic> gyr_measurements;

  std::tie(stamps, gyr_measurements) =
      gyr_buffer_.getBetweenValuesInterpolated<this.template Interpolator>(
        stamp_from, stamp_to);

  // resample the accelerometer measurement at the gyro timestamps
  imu_measurements.topRows<3>() = acc_buffer_.getValuesInterpolated(stamps);
  imu_measurements.bottomRows<3>() = gyr_measurements;

  return std::make_pair(stamps, imu_measurements);
}

} // namespace ze
