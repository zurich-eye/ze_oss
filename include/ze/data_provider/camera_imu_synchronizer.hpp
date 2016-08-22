#pragma once

#include <memory>
#include <imp/core/image_base.hpp>
#include <ze/common/ringbuffer.h>
#include <ze/common/time_conversions.h>
#include <ze/common/types.h>
#include <ze/data_provider/camera_imu_synchronizer_base.hpp>

namespace ze {

// fwd
class DataProviderBase;
class ImageBase;

// -----------------------------------------------------------------------------
class CameraImuSynchronizer: public CameraImuSynchronizerBase
{
public:
  // convenience typedefs
  using ImuSyncBuffer = Ringbuffer<real_t, 6, 1000>;
  using ImuBufferVector = std::vector<ImuSyncBuffer>;

  //! Default constructor.
  CameraImuSynchronizer(DataProviderBase& data_provider);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(
      int64_t stamp,
      const Vector3& acc,
      const Vector3& gyr,
      const uint32_t imu_idx);

private:
  //! Register callbacks in data provider to this class' addImgData and addImuData.
  void subscribeDataProvider(DataProviderBase& data_provider);

  //! IMU buffer stores all imu measurements, size of imu_count_.
  ImuBufferVector imu_buffers_;

  //! Initialize the image and imu buffers
  void initBuffers();

  //! This function checks if we have all data ready to call the callback.
  virtual void checkImuDataAndCallback();
};

} // namespace ze
