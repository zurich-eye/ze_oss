#pragma once

#include <memory>
#include <imp/core/image_base.hpp>
#include <ze/imu/imu_buffer.h>
#include <ze/common/types.h>
#include <ze/common/time_conversions.h>

namespace ze {

class DataProviderBase;

// convenience typedefs
using ImuStampsVector = std::vector<ImuStamps>;
using ImuAccGyrVector = std::vector<ImuAccGyrContainer>;

// callback typedefs
using SynchronizedCameraImuCallback =
  std::function<void (const StampedImages& /*images*/,
                      const ImuStampsVector& /*imu_timestamps*/,
                      const ImuAccGyrVector& /*imu_measurements*/)>;

// -----------------------------------------------------------------------------
//! Container to buffer received images.
struct ImageBufferItem
{
  int64_t stamp        { -1 };
  ImageBasePtr img     { nullptr };
  int32_t camera_idx   { -1 };
  inline bool empty()
  {
    return stamp == -1;
  }

  inline void reset()
  {
    stamp = -1;
    img.reset();
    camera_idx = -1;
  }
};
using ImgBuffer = std::vector<ImageBufferItem>;

class CameraImuSynchronizerBase
{
public:
  //! Default constructor.
  CameraImuSynchronizerBase(DataProviderBase& data_provider);

  void registerCameraImuCallback(const SynchronizedCameraImuCallback& callback);

  //! Add Image to the frame synchronizer.
  void addImgData(
      int64_t stamp,
      const ImageBase::Ptr& img,
      uint32_t camera_idx);

protected:
  //! Allowed time differences of images in bundle.
  static constexpr FloatType c_camera_bundle_time_accuracy_ns = millisecToNanosec(2.0);

  //! Stamp of previous synchronized image bundle.
  int64_t last_img_bundle_min_stamp_ { -1 };

  //! Num images to synchronize.
  uint32_t num_cameras_;

  //! Num IMUs to synchronize.
  uint32_t num_imus_;

  StampedImages sync_imgs_ready_to_process_;
  int64_t sync_imgs_ready_to_process_stamp_ { -1 };

  //! Image buffer is buffering images of max the last 2*rig_size images.
  ImgBuffer img_buffer_;

  //! This function checks if we have all data ready to call the callback.
  virtual void checkImuDataAndCallback() = 0;

  //! Validate the contents of the IMU buffer relative to the image buffers.
  bool validateImuBuffers(
      const int64_t& min_stamp,
      const int64_t& max_stamp,
      const std::vector<std::tuple<int64_t, int64_t, bool>>& oldest_newest_stamp_vector);

  //! Max time difference of images in a bundle
  int64_t img_bundle_max_dt_nsec_ = millisecToNanosec(2.0);

  //! Count number of synchronized frames.
  int sync_frame_count_  { 0 };

  //! Registered callback for synchronized measurements.
  SynchronizedCameraImuCallback cam_imu_callback_;
};

} // namespace ze
