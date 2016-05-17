#include <ze/data_provider/camera_imu_synchronizer.h>
#include <ze/data_provider/data_provider_base.h>

#include <functional>

#include <ze/common/logging.hpp>
#include <gflags/gflags.h>

DEFINE_int32(data_sync_init_skip_n_frames, 0,
             "How many frames should be skipped at the beginning.");

DEFINE_int32(data_sync_stop_after_n_frames, -1,
             "How many frames should be processed?");

namespace ze {

CameraImuSynchronizer::CameraImuSynchronizer(
    DataProviderBase& data_provider,
    FloatType imu_buffer_length_seconds)
  : camera_rig_size_(data_provider.cameraCount())
  , imu_count_(data_provider.imuCount())
{
  subscribeDataProvider(data_provider);
  initBuffers(imu_buffer_length_seconds);
}

void CameraImuSynchronizer::subscribeDataProvider(DataProviderBase& data_provider)
{
  using namespace std::placeholders;
  if (camera_rig_size_ == 0u)
  {
    LOG(ERROR) << "DataProvider must at least expose a single camera topic.";
  }
  data_provider.registerCameraCallback(
        std::bind(&CameraImuSynchronizer::addImgData, this, _1, _2, _3));

  if (imu_count_ > 0u)
  {
    data_provider.registerImuCallback(
          std::bind(&CameraImuSynchronizer::addImuData, this, _1, _2, _3, _4));
  }
}

void CameraImuSynchronizer::initBuffers(FloatType imu_buffer_length_seconds)
{
  img_buffer_ = StampedImages(camera_rig_size_, std::make_pair(-1, nullptr));
  imu_buffers_ = ImuBufferVector(imu_count_, Buffer<FloatType, 6>(imu_buffer_length_seconds));
}

void CameraImuSynchronizer::addImgData(
    int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx)
{
  CHECK_LT(camera_idx, camera_rig_size_);
  if(img_buffer_.at(camera_idx).first != -1)
  {
    LOG(WARNING) << "Received new camera image before the previous set was complete."
                 << " Unordered camera images are arriving!";
  }

  // time consistency checks
  // ensure that the oldest image is < 2 ms offset from current image (same frame)
  for (size_t i = 0; i < img_buffer_.size(); ++i)
  {
    if (i == camera_idx)
    {
      continue;
    }

    if (img_buffer_.at(i).first != -1 && img_buffer_.at(i).first > stamp)
    {
      LOG(WARNING) << "Unordered images arriving.";
    }

    if (img_buffer_.at(i).first != -1 && std::abs(stamp - img_buffer_.at(i).first) > 2000)
    {
      // invalidate old data
      img_buffer_.at(i).first = -1;
      LOG(WARNING) << "Invalidated image of older frame.";
    }
  }

  if (camera_idx == 0)
  {
    ++sync_frame_count_;
    if (sync_frame_count_ < FLAGS_data_sync_init_skip_n_frames)
    {
      return;
    }
  }

  img_buffer_[camera_idx] = std::make_pair(stamp, img);
  checkDataAndCallback();
}

void CameraImuSynchronizer::addImuData(
    int64_t stamp, const Vector3& acc, const Vector3& gyr, const uint32_t imu_idx)
{
  Vector6 acc_gyr;
  acc_gyr.head<3>() = acc;
  acc_gyr.tail<3>() = gyr;
  imu_buffers_.at(imu_idx).insert(stamp, acc_gyr);
  checkDataAndCallback();
}

void CameraImuSynchronizer::registerCameraImuCallback(
    const SynchronizedCameraImuCallback& callback)
{
  cam_imu_callback_ = callback;
}

bool CameraImuSynchronizer::validateImuBuffers(
    const int64_t& min_stamp,
    const int64_t& max_stamp,
    const std::vector<std::tuple<int64_t, int64_t, bool> >&
      oldest_newest_stamp_vector)
{
  // Check if we have received some IMU measurements for at least one of the imu's.
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                   {
                     if (std::get<2>(oldest_newest_stamp))
                     {
                       return true;
                     }
                     return false;
                   })
      )
  {
    LOG(WARNING) << "Received all images but no imu measurements!";
    resetImgBuffer();
    return false;
  }

  // At least one IMU measurements before image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&min_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp)
                   {
                     if (std::get<0>(oldest_newest_stamp) < min_stamp) {
                       return true;
                     }
                     return false;
                   })
  )
  {
    LOG(WARNING) << "Oldest IMU measurement is newer than image timestamp.";
    resetImgBuffer();
    return false;
  }

  // At least one IMU measurements after image
  if (std::none_of(oldest_newest_stamp_vector.begin(),
                   oldest_newest_stamp_vector.end(),
                   [&max_stamp](const std::tuple<int64_t, int64_t, bool>& oldest_newest_stamp) {
                     if (std::get<1>(oldest_newest_stamp) > max_stamp)
                     {
                       return true;
                     }
                     return false;
                   })
  )
  {
    VLOG(100) << "Waiting for IMU measurements.";
    return false;
  }

  return true;
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
                 << nanosecToMillisecTrunc(max_stamp - min_stamp) << " milliseconds";
  }

  // always provide imu structures in the callback (empty if no imu present)
  ImuStampsVector imu_timestamps(imu_buffers_.size());
  ImuAccGyrVector imu_measurements(imu_buffers_.size());

  if (imu_count_ != 0)
  {
    // get oldest / newest stamp for all imu buffers
    std::vector<std::tuple<int64_t, int64_t, bool> > oldest_newest_stamp_vector(imu_buffers_.size());
    std::transform(
          imu_buffers_.begin(),
          imu_buffers_.end(),
          oldest_newest_stamp_vector.begin(),
          [](Buffer<FloatType, 6>& imu_buffer) {
            return imu_buffer.getOldestAndNewestStamp();
          }
    );

    // imu buffers are not consistent with the image buffers
    if (!validateImuBuffers(min_stamp, max_stamp, oldest_newest_stamp_vector))
    {
      return;
    }

    // If this is the very first image bundle, we send all IMU messages that we have
    // received so far. For every later image bundle, we just send the IMU messages
    // that we have received in between.
    for (size_t i = 0; i < imu_buffers_.size(); ++i)
    {
      if(last_img_bundle_min_stamp_ < 0)
      {
        int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);

        ImuStamps imu_stamps;
        ImuAccGyr imu_accgyr;
        std::tie(imu_stamps, imu_accgyr) =
            imu_buffers_[i].getBetweenValuesInterpolated(oldest_stamp, min_stamp);
        imu_timestamps[i] = imu_stamps;
        imu_measurements[i] = imu_accgyr;
      }
      else
      {
        ImuStamps imu_stamps;
        ImuAccGyr imu_accgyr;
        std::tie(imu_stamps, imu_accgyr) =
            imu_buffers_[i].getBetweenValuesInterpolated(last_img_bundle_min_stamp_, min_stamp);
        imu_timestamps[i] = imu_stamps;
        imu_measurements[i] = imu_accgyr;
      }
    }
  }

  // Let's process the callback.
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
