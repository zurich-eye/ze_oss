#pragma once

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/time.h>
#include <ze/data_provider/data_provider_base.h>
#include <imp/core/image_base.hpp>

namespace ze {

using StampedImage = std::pair<int64_t, ImageBase::Ptr>;
using StampedImages = std::vector<StampedImage>;
using SynchronizedCameraImuCallback =
  std::function<void (const StampedImages& /*images*/,
                      const ImuStamps& /*imu_timestamps*/,
                      const ImuAccGyr& /*imu_measurements*/)>;

class FrameImuSynchronizer
{
public:
  FrameImuSynchronizer(uint32_t num_frames,
                       FloatType imu_buffer_length_seconds);


  //! Add Image to the frame synchronizer.
  void addImgData(int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(int64_t stamp, const Vector3& acc, const Vector3& gyr);

  //! Register callbacks in data provider to this class' addImgData and addImuData.
  void subscribeDataProvider(const DataProviderBase::Ptr& data_provider);

  //! When we have received all camera and IMU measurements since the last frame
  //! the callback that is registered here will be called.
  void registerCameraImuCallback(const SynchronizedCameraImuCallback& callback);

private:

  //! Num images to synchronize.
  uint32_t camera_rig_size_;

  //! Max time difference of images in a bundle
  int64_t img_bundle_max_dt_nsec_ = millisecToNanosec(2.0);

  //! Stamp of previous synchronized image bundle.
  int64_t last_img_bundle_min_stamp_ = -1;

  //! Image buffer has fixed size of num_frames.
  StampedImages img_buffer_;

  //! IMU buffer stores all imu measurements.
  Buffer<FloatType, 6> imu_buffer_;

  //! Registered callback.
  SynchronizedCameraImuCallback cam_imu_callback_;

  //! This function checks if we have all data ready to call the callback.
  void checkDataAndCallback();

  //! Clear all
  void resetImgBuffer();
};

} // namespace ze
