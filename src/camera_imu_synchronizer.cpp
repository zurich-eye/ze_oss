#include <ze/data_provider/camera_imu_synchronizer.h>
#include <ze/data_provider/data_provider_base.h>

#include <functional>

#include <ze/common/logging.hpp>

namespace ze {

CameraImuSynchronizer::CameraImuSynchronizer(
    uint32_t num_frames, FloatType imu_buffer_length_seconds)
  : camera_rig_size_(num_frames)
  , img_buffer_(num_frames, std::make_pair(-1, nullptr))
  , imu_buffer_(imu_buffer_length_seconds)
{}

void CameraImuSynchronizer::addImgData(
    int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx)
{
  CHECK_LT(camera_idx, camera_rig_size_);
  if(img_buffer_.at(camera_idx).first != -1)
  {
    LOG(WARNING) << "Received new camera image before the previous set was complete."
                 << " Unordered camera images are arriving!";
  }
  img_buffer_[camera_idx] = std::make_pair(stamp, img);
  checkDataAndCallback();
}

void CameraImuSynchronizer::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr)
{
  Vector6 acc_gyr;
  acc_gyr.head<3>() = acc;
  acc_gyr.tail<3>() = gyr;
  imu_buffer_.insert(stamp, acc_gyr);
  checkDataAndCallback();
}

void CameraImuSynchronizer::subscribeDataProvider(
    DataProviderBase& data_provider)
{
  using namespace std::placeholders;
  data_provider.registerCameraCallback(
        std::bind(&CameraImuSynchronizer::addImgData, this, _1, _2, _3));
  data_provider.registerImuCallback(
        std::bind(&CameraImuSynchronizer::addImuData, this, _1, _2, _3));
}

void CameraImuSynchronizer::registerCameraImuCallback(
    const SynchronizedCameraImuCallback& callback)
{
  cam_imu_callback_ = callback;
}

void CameraImuSynchronizer::checkDataAndCallback()
{
  // Check if we have received all images from the cameras:
  int64_t max_stamp = std::numeric_limits<int64_t>::min();
  int64_t min_stamp = std::numeric_limits<int64_t>::max();
  for(const StampedImage& it : img_buffer_)
  {
    if(it.first < 0)
    {
      // Negative time-stamp means that this image was not yet received!
      return;
    }
    max_stamp = std::max(it.first, max_stamp);
    min_stamp = std::min(it.first, min_stamp);
  }

  if(max_stamp - min_stamp > img_bundle_max_dt_nsec_)
  {
    LOG(WARNING) << "Images in bundle have too large varying timestamps: "
                 << nanosecToMillisec(max_stamp - min_stamp) << " milliseconds";
  }

  // Check if we have received some IMU measurements.
  std::tuple<int64_t, int64_t, bool> oldest_newest_stamp =
      imu_buffer_.getOldestAndNewestStamp();
  if(!std::get<2>(oldest_newest_stamp))
  {
    LOG(WARNING) << "Received all images but no imu measurements!";
    resetImgBuffer();
    return;
  }

  // Check that oldest IMU measurement is not newer than the image stamp.
  if(std::get<0>(oldest_newest_stamp) > min_stamp)
  {
    LOG(WARNING) << "Oldest IMU measurement is newer than image timestamp.";
    resetImgBuffer();
    return;
  }

  // Check if we have IMU measurements until the latest image.
  if(std::get<1>(oldest_newest_stamp) < max_stamp)
  {
    VLOG(100) << "Waiting for IMU measurements.";
    return;
  }

  // If this is the very first image bundle, we send all IMU messages that we have
  // received so far. For every later image bundle, we just send the IMU messages
  // that we have received in between.
  ImuStamps imu_timestamps;
  ImuAccGyr imu_measurements;
  if(last_img_bundle_min_stamp_ < 0)
  {
    int64_t oldest_stamp = std::get<0>(oldest_newest_stamp);
    std::tie(imu_timestamps, imu_measurements) =
        imu_buffer_.getBetweenValuesInterpolated(oldest_stamp, min_stamp);
  }
  else
  {
    std::tie(imu_timestamps, imu_measurements) =
        imu_buffer_.getBetweenValuesInterpolated(last_img_bundle_min_stamp_, min_stamp);
  }

  // Awesome, we have all data that we need, let's process the callback.
  cam_imu_callback_(img_buffer_, imu_timestamps, imu_measurements);

  last_img_bundle_min_stamp_ = min_stamp;
  resetImgBuffer();
}

void CameraImuSynchronizer::resetImgBuffer()
{
  for(StampedImage& it : img_buffer_)
  {
    it.first = -1;
    it.second.reset();
  }
}

} // namespace ze
