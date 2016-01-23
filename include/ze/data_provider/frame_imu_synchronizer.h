#pragma once

#include <ze/common/types.h>
#include <ze/common/buffer.h>
#include <imp/core/image_base.hpp>

namespace ze {

using StampedImage = std::pair<int64_t, ImageBase::Ptr>;
using StampedImages = std::vector<StampedImage>;
using SynchronizedCameraImuCallback =
  std::function<void (const StampedImages& /*images*/,
                      const ImuStamps& /*imu_stamps*/,
                      const ImuAccGyr& /*imu_measurements*/)>;

class FrameImuSynchronizer
{
public:
  FrameImuSynchronizer(uint32_t num_frames,
                       FloatType imu_buffer_length_seconds);


  //! Add Image to the frame synchronizer.
  void addImgData(int64_t stamp, const Vector3& acc, const Vector3& gyr);

  //! Add IMU measurement to the frame synchronizer.
  void addImuData(int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx);

  //! When we have received all camera and IMU measurements since the last frame
  //! the callback that is registered here will be called.
  void registerCameraImuCallback(const SynchronizedCameraImuCallback& callback);

private:

  //! Image buffer has fixed size of num_frames.
  StampedImages img_buffer_;

  //! IMU buffer stores all imu measurements
  Buffer<FloatType, 6> imu_buffer_;
};


} // namespace ze
