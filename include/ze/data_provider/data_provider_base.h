#pragma once

#include <atomic>
#include <memory>
#include <functional>

#include <imp/core/image.hpp>
#include <ze/common/macros.h>
#include <ze/common/signal_handler.hpp>
#include <ze/common/types.h>
#include <ze/common/noncopyable.hpp>

// fwd
namespace cv {
class Mat;
}

namespace ze {

using ImuCallback =
  std::function<void (int64_t /*stamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/,
                      uint32_t /*imu-idx*/)>;
using CameraCallback =
  std::function<void (int64_t /*stamp*/,
                      const ImageBase::Ptr& /*img*/,
                      uint32_t /*camera-idx*/)>;

enum class DataProviderType {
  Csv,
  Rosbag
};

class DataProviderBase : Noncopyable
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

  virtual size_t imu_count() const = 0;
  virtual size_t camera_count() const = 0;

  void registerImuCallback(const ImuCallback& imu_callback);

  void registerCameraCallback(const CameraCallback& camera_callback);

protected:
  DataProviderType type_;
  ImuCallback imu_callback_;
  CameraCallback camera_callback_;
  volatile bool running_ = true;

private:
  SimpleSigtermHandler signal_handler_; //!< Sets running_ to false when Ctrl-C is pressed.
};

} // namespace ze
