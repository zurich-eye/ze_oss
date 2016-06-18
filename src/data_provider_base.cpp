#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

DataProviderBase::DataProviderBase(DataProviderType type)
  : type_(type)
  , signal_handler_(running_)
{}

void DataProviderBase::spin()
{
  while (ok())
  {
    spinOnce();
  }
}

void DataProviderBase::pause()
{
  running_ = false;
}

void DataProviderBase::shutdown()
{
  running_ = false;
}

void DataProviderBase::registerImuCallback(const ImuCallback& imu_callback)
{
  imu_callback_ = imu_callback;
}

void DataProviderBase::registerGyroCallback(const GyroCallback& gyro_callback)
{
  gyro_callback_ = gyro_callback;
}

void DataProviderBase::registerAccelCallback(const AccelCallback& accel_callback)
{
  accel_callback_ = accel_callback;
}

void DataProviderBase::registerCameraCallback(const CameraCallback& camera_callback)
{
  camera_callback_ = camera_callback;
}

} // namespace ze
