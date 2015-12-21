#pragma once

#include <memory>
#include <functional>

#include <ze/common/macros.h>
#include <ze/common/types.h>

// fwd
namespace cv {
class Mat;
}

namespace ze {
namespace data_provider {

using ImuCallback =
  std::function<void (int64_t /*stamp*/, const Eigen::Vector3d& /*acc*/, const Eigen::Vector3d& /*gyr*/)>;

using CameraCallback =
  std::function<void (int64_t /*stamp*/, const cv::Mat& /*img*/, size_t /*camera-idx*/)>;

} // namespace data_provider

class DataProviderBase
{
public:
  ZE_POINTER_TYPEDEFS(DataProviderBase);

  DataProviderBase() = default;
  virtual ~DataProviderBase() = default;

  // Read next data field and process callback. Waits until callback is processed.
  virtual void spinOnceBlocking() = 0;

  inline void registerImuCallback(const data_provider::ImuCallback& imu_callback)
  {
    imu_callback_ = imu_callback;
  }

  inline void registerCameraCallback(const data_provider::CameraCallback& camera_callback)
  {
    camera_callback_ = camera_callback;
  }

protected:
  data_provider::ImuCallback imu_callback_;
  data_provider::CameraCallback camera_callback_;
};

} // namespace ze
