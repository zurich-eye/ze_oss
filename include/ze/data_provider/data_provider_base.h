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

using ImuCallback =
  std::function<void (int64_t /*stamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/)>;
using CameraCallback =
  std::function<void (int64_t /*stamp*/,
                      const cv::Mat& /*img*/,
                      size_t /*camera-idx*/)>;

enum class DataProviderType {
  Csv,
  Rosbag
};

class DataProviderBase
{
public:
  ZE_POINTER_TYPEDEFS(DataProviderBase);

  DataProviderBase(DataProviderType type);
  virtual ~DataProviderBase() = default;

  //! Process all callbacks. Waits until callback is processed.
  virtual void spin() = 0;

  //! Read next data field and process callback. Returns false when datatset finished.
  virtual bool spinOnce() = 0;

  //! False if there is no more data to process or there was a shutdown signal.
  virtual bool ok() const = 0;

  //! Stop data provider.
  virtual void shutdown();

  void registerImuCallback(const ImuCallback& imu_callback);

  void registerCameraCallback(const CameraCallback& camera_callback);

protected:
  DataProviderType type_;
  ImuCallback imu_callback_;
  CameraCallback camera_callback_;
  std::atomic_bool shutdown_;
};

} // namespace ze
