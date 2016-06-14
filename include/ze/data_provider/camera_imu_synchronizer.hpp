#pragma once

#include <memory>
#include <ze/common/ringbuffer.h>
#include <ze/common/types.h>
#include <ze/common/time_conversions.h>

namespace ze {

// fwd
class DataProviderBase;
class ImageBase;

// convenience typedefs
using ImuStampsVector = std::vector<ImuStamps>;
using ImuAccGyrVector = std::vector<ImuAccGyr>;
using ImuSyncBuffer = Ringbuffer<FloatType, 6, 1000>;
using ImuBufferVector = std::vector<ImuSyncBuffer>;
using ImageBasePtr = std::shared_ptr<ImageBase>;
using StampedImage = std::pair<int64_t, ImageBasePtr>;
using StampedImages = std::vector<StampedImage>;

// callback typedefs
using SynchronizedCameraImuCallback =
  std::function<void (const StampedImages& /*images*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;

//! Synchronizes multiple cameras with multiple imus. Triggers a callback
//! once measurements from all cameras and IMUs are available.
class CameraImuSynchronizer
{
public:
  //! Default constructor.
  CameraImuSynchronizer(DataProviderBase& data_provider);

  //! Add Image to the frame synchronizer.
  void addImgData(
      int64_t stamp,
      const ImageBasePtr& img,
      uint32_t camera_idx);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(
      int64_t stamp,
      const Vector3& acc,
      const Vector3& gyr,
      const uint32_t imu_idx);

  //! When we have received all camera and IMU measurements since the last frame
  //! the callback that is registered here will be called.
  void registerCameraImuCallback(const SynchronizedCameraImuCallback& callback);

private:
  //! Num images to synchronize.
  uint32_t num_cameras_;

  //! Num IMUs to synchronize.
  uint32_t num_imus_;

  //! Max time difference of images in a bundle
  int64_t img_bundle_max_dt_nsec_ = millisecToNanosec(2.0);

  //! Stamp of previous synchronized image bundle.
  int64_t last_img_bundle_min_stamp_ { -1 };

  //! Count number of synchronized frames.
  int sync_frame_count_  { 0 };

  //! Image buffer has fixed size of camera_rig_size_.
  StampedImages img_buffer_;

  //! IMU buffer stores all imu measurements, size of imu_count_.
  ImuBufferVector imu_buffers_;

  //! Registered callback for synchronized measurements.
  SynchronizedCameraImuCallback cam_imu_callback_;

  //! Register callbacks in data provider to this class' addImgData and addImuData.
  void subscribeDataProvider(DataProviderBase& data_provider);

  //! Initialize the image and imu buffers
  void initBuffers();

  //! Validate the contents of the IMU buffer relative to the image buffers.
  bool validateImuBuffers(
      const int64_t& min_stamp,
      const int64_t& max_stamp,
      const std::vector<std::tuple<int64_t, int64_t, bool>>& oldest_newest_stamp_vector);

  //! This function checks if we have all data ready to call the callback.
  void checkDataAndCallback();

  //! Clear all
  void resetImgBuffer();
};

} // namespace ze
