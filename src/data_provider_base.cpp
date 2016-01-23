#include <ze/data_provider/data_provider_base.h>

namespace ze {

DataProviderBase::DataProviderBase(DataProviderType type)
  : type_(type)
  , shutdown_(false)
{}

void DataProviderBase::shutdown()
{
  // TODO(cfo): Catch SIGINT signal.
  shutdown_ = true;
}

void DataProviderBase::registerImuCallback(const ImuCallback& imu_callback)
{
  imu_callback_ = imu_callback;
}

void DataProviderBase::registerCameraCallback(const CameraCallback& camera_callback)
{
  camera_callback_ = camera_callback;
}

} // namespace ze
