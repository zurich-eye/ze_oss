#include <ze/imu/imu_buffer.h>

namespace ze {

template<int BufferSize, typename Interpolator>
ImuBuffer<BufferSize, Interpolator>::ImuBuffer(ImuModel::Ptr imu_model)
  : imu_model_(imu_model)
{
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertImu(
    int64_t time, const Vector6 value)
{
  acc_buffer_.insert(time, value.head<3>(3));
  gyr_buffer_.insert(time, value.tail<3>(3));
}


template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertGyroscope(
    int64_t time, const Vector3 value)
{
  gyr_buffer_.insert(time, value);
}

template<int BufferSize, typename Interpolator>
void ImuBuffer<BufferSize, Interpolator>::insertAccelerometer(
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
std::pair<Eigen::Matrix<time_t, Eigen::Dynamic, 1>,
Eigen::Matrix<FloatType, 6, Eigen::Dynamic> >
ImuBuffer<BufferSize, Interpolator>::getBetweenValuesInterpolated(
    int64_t stamp_from, int64_t stamp_to)
{
  return std::make_pair<Eigen::Matrix<time_t, Eigen::Dynamic, 1>,
      Eigen::Matrix<FloatType, 6, Eigen::Dynamic>>(MatrixX(), MatrixX());
}

} // namespace ze
