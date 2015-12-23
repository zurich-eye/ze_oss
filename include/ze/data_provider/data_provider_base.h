#pragma once

#include <atomic>
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

  DataProviderBase();
  virtual ~DataProviderBase() = default;

  // Process all callbacks.
  virtual void spin() = 0;

  // Read next data field and process callback. Waits until callback is processed.
  // Returns false when datatset finished.
  virtual bool spinOnce() = 0;

  // False if there is no more data to process or there was a shutdown signal.
  virtual bool ok() const = 0;

  // Stop data provider.
  virtual void shutdown();

  void registerImuCallback(const data_provider::ImuCallback& imu_callback);

  void registerCameraCallback(const data_provider::CameraCallback& camera_callback);

protected:
  data_provider::ImuCallback imu_callback_;
  data_provider::CameraCallback camera_callback_;
  std::atomic_bool shutdown_;
};

} // namespace ze
