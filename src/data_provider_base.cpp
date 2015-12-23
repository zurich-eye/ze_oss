#include <ze/data_provider/data_provider_base.h>

namespace ze {

DataProviderBase::DataProviderBase(data_provider::Type type)
  : type_(type)
  , shutdown_(false)
{}

void DataProviderBase::shutdown()
{
  // TODO(cfo): Catch SIGINT signal.
  shutdown_ = true;
}

void DataProviderBase::registerImuCallback(const data_provider::ImuCallback& imu_callback)
{
  imu_callback_ = imu_callback;
}

void DataProviderBase::registerCameraCallback(const data_provider::CameraCallback& camera_callback)
{
  camera_callback_ = camera_callback;
}

} // namespace ze
