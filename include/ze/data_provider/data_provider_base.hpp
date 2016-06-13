#pragma once

#include <atomic>
#include <memory>
#include <functional>

#include <ze/common/macros.h>
#include <ze/common/signal_handler.hpp>
#include <ze/common/types.h>
#include <ze/common/noncopyable.hpp>

// fwd
namespace cv {
class Mat;
}

namespace ze {

// fwd
class ImageBase;

using ImuCallback =
  std::function<void (int64_t /*timestamp*/,
                      const Vector3& /*acc*/,
                      const Vector3& /*gyr*/,
                      uint32_t /*imu-idx*/)>;
using CameraCallback =
  std::function<void (int64_t /*timestamp*/,
                      const std::shared_ptr<ImageBase>& /*img*/,
                      uint32_t /*camera-idx*/)>;

enum class DataProviderType {
  Csv,
  Rosbag,
  Rostopic
};

//! A data provider registers to a data source and triggers callbacks when
//! new data is available.
class DataProviderBase : Noncopyable
{
public:
  ZE_POINTER_TYPEDEFS(DataProviderBase);

  DataProviderBase() = delete;
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

  //! Number of imus to process.
  virtual size_t imuCount() const = 0;

  //! Number of cameras to process.
  virtual size_t cameraCount() const = 0;

  //! Register callback function to call when new IMU message is available.
  void registerImuCallback(const ImuCallback& imu_callback);

  //! Register callback function to call when new camera message is available.
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
