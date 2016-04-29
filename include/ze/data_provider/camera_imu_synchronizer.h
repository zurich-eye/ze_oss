#pragma once

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <ze/common/time_conversions.h>
#include <imp/core/image_raw.hpp>

namespace ze {

// fwd
class DataProviderBase;

using StampedImage = std::pair<int64_t, ImageBase::Ptr>;
using StampedImages = std::vector<StampedImage>;
using ImuStampsVector = std::vector<ImuStamps>;
using ImuAccGyrVector = std::vector<ImuAccGyr>;
using ImuBufferVector = std::vector<Buffer<FloatType, 6> >;

using SynchronizedCameraImuCallback =
  std::function<void (const StampedImages& /*images*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;

class CameraImuSynchronizer
{
public:
  CameraImuSynchronizer(DataProviderBase& data_provider, FloatType imu_buffer_length_seconds);


  //! Add Image to the frame synchronizer.
  void addImgData(int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx);

  //! When we have received all camera and IMU measurements since the last frame
  //! the callback that is registered here will be called.
  void registerCameraImuCallback(const SynchronizedCameraImuCallback& callback);

private:

  //! Num images to synchronize.
  uint32_t camera_rig_size_;

  //! Num imus to synchronize.
  uint32_t imu_count_;

  //! Max time difference of images in a bundle
  int64_t img_bundle_max_dt_nsec_ = millisecToNanosec(2.0);

  //! Stamp of previous synchronized image bundle.
  int64_t last_img_bundle_min_stamp_ = -1;

  //! Count number of synchronized frames.
  int sync_frame_count_ = 0;

  //! Image buffer has fixed size of camera_rig_size_.
  StampedImages img_buffer_;

  //! IMU buffer stores all imu measurements, size of imu_count_.
  ImuBufferVector imu_buffers_;

  //! Registered callback.
  SynchronizedCameraImuCallback cam_imu_callback_;

  //! Register callbacks in data provider to this class' addImgData and addImuData.
  void subscribeDataProvider(DataProviderBase& data_provider);

  //! Initialize the image and imu buffers
  void initBuffers(FloatType imu_buffer_length_seconds);

  //! Validate the contents of the IMU buffer relative to the image buffers
  bool validateImuBuffers(const int64_t& min_stamp,
                          const int64_t& max_stamp,
                          const std::vector<std::tuple<int64_t, int64_t, bool> >& oldest_newest_stamp_vector);

  //! This function checks if we have all data ready to call the callback.
  void checkDataAndCallback();

  //! Clear all
  void resetImgBuffer();
};

} // namespace ze
