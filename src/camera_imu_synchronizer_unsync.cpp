#include <ze/data_provider/camera_imu_synchronizer_unsync.hpp>

#include <functional>
#include <gflags/gflags.h>

#include <ze/common/logging.hpp>
#include <ze/data_provider/data_provider_base.hpp>

DEFINE_int32(data_sync_init_skip_n_frames_unsync, 0,
             "How many frames should be skipped at the beginning.");

namespace ze {

CameraImuSynchronizerUnsync::CameraImuSynchronizerUnsync(
    DataProviderBase& data_provider,
    const std::vector<ImuModel::Ptr>& imu_models)
  : num_cameras_(data_provider.cameraCount())
  , num_imus_(data_provider.imuCount())
{
  subscribeDataProvider(data_provider);
  initBuffers(imu_models);
}

void CameraImuSynchronizerUnsync::subscribeDataProvider(
    DataProviderBase& data_provider)
{
  using namespace std::placeholders;
  if (num_cameras_ == 0u)
  {
    LOG(ERROR) << "DataProvider must at least expose a single camera topic.";
  }
  data_provider.registerCameraCallback(
        std::bind(&CameraImuSynchronizerUnsync::addImgData, this, _1, _2, _3));

  if (num_imus_ > 0u)
  {
    data_provider.registerAccelCallback(
          std::bind(&CameraImuSynchronizerUnsync::addAccelData, this, _1, _2, _3));
    data_provider.registerGyroCallback(
          std::bind(&CameraImuSynchronizerUnsync::addGyroData, this, _1, _2, _3));
  }
}

void CameraImuSynchronizerUnsync::initBuffers(
    const std::vector<ImuModel::Ptr>& imu_models)
{
  img_buffer_.resize(2 * num_cameras_);
  for (ImuModel::Ptr imu_model: imu_models)
  {
    imu_buffers_.push_back(std::make_shared<ImuSyncBufferUnsync>(imu_model));
  }
}

void CameraImuSynchronizerUnsync::addImgData(
    int64_t stamp, const ImageBase::Ptr& img, uint32_t camera_idx)
{
  CHECK_LT(camera_idx, num_cameras_);

  // Skip frame processing for first N frames.
  if (sync_frame_count_ < FLAGS_data_sync_init_skip_n_frames_unsync)
  {
    if (camera_idx == 0)
    {
      ++sync_frame_count_;
    }
    return;
  }

  // Add image to first available slot in our buffer:
  int slot = -1;
  for (size_t i = 0u; i < img_buffer_.size(); ++i)
  {
    if (img_buffer_[i].empty())
    {
      slot = i;
      break;
    }
  }

  if (slot == -1)
  {
    // No space in buffer to process frame. Delete oldest one. This happens
    // also when the processing is not fast enough such that frames are skipped.
    int64_t min_stamp = std::numeric_limits<int64_t>::max();
    for (size_t i = 0u; i < img_buffer_.size(); ++i)
    {
      if (!img_buffer_[i].empty() && img_buffer_[i].stamp < min_stamp)
      {
        slot = i;
        min_stamp = img_buffer_[i].stamp;
      }
    }
  }

  img_buffer_[slot].stamp = stamp;
  img_buffer_[slot].img = img;
  img_buffer_[slot].camera_idx = camera_idx;

  // Now check, if we have all images from this bundle:
  uint32_t num_imgs = 0u;
  for (size_t i = 0; i <= img_buffer_.size(); ++i)
  {
    if (std::abs(stamp - img_buffer_[i].stamp) < millisecToNanosec(2))
    {
      ++num_imgs;
    }
  }

  if (num_imgs != num_cameras_)
  {
    return; // We don't have all frames yet.
  }

  // We have frames with very close timestamps. Put them together in a vector.
  sync_imgs_ready_to_process_.clear();
  sync_imgs_ready_to_process_.resize(num_imgs, {-1, nullptr});
  for (size_t i = 0; i <= img_buffer_.size(); ++i)
  {
    ImageBufferItem& item = img_buffer_[i];
    if (std::abs(stamp - item.stamp) < c_camera_bundle_time_accuracy_ns)
    {
      DEBUG_CHECK_GT(item.stamp, 0);
      DEBUG_CHECK(item.img);
      sync_imgs_ready_to_process_.at(item.camera_idx) = { item.stamp, item.img };
    }
  }

  // Double-check that we have all images.
  for (size_t i = 0; i < sync_imgs_ready_to_process_.size(); ++i)
  {
    if (sync_imgs_ready_to_process_.at(i).first == -1)
    {
      LOG(ERROR) << "Sync images failed!";
      sync_imgs_ready_to_process_.clear();
      return;
    }
  }
  sync_imgs_ready_to_process_stamp_ = sync_imgs_ready_to_process_.front().first;

  checkImuDataAndCallback();
}

void CameraImuSynchronizerUnsync::addAccelData(
    int64_t stamp, const Vector3& acc, const uint32_t imu_idx)
{
  imu_buffers_[imu_idx]->insertAccelerometerMeasurement(stamp, acc);
  checkImuDataAndCallback();
}

void CameraImuSynchronizerUnsync::addGyroData(
    int64_t stamp, const Vector3& gyr, const uint32_t imu_idx)
{
  imu_buffers_[imu_idx]->insertGyroscopeMeasurement(stamp, gyr);
  checkImuDataAndCallback();
}

void CameraImuSynchronizerUnsync::registerCameraImuCallback(
    const SynchronizedCameraImuCallback& callback)
{
  cam_imu_callback_ = callback;
}

bool CameraImuSynchronizerUnsync::validateImuBuffers(
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

void CameraImuSynchronizerUnsync::checkImuDataAndCallback()
{
  if (sync_imgs_ready_to_process_stamp_ < 0)
  {
    return; // Images are not synced yet.
  }

  // always provide imu structures in the callback (empty if no imu present)
  ImuStampsVector imu_timestamps(num_imus_);
  ImuAccGyrVector imu_measurements(num_imus_);

  if (num_imus_ != 0)
  {
    // get oldest / newest stamp for all imu buffers
    std::vector<std::tuple<int64_t, int64_t, bool>> oldest_newest_stamp_vector(num_imus_);
    std::transform(
          imu_buffers_.begin(),
          imu_buffers_.end(),
          oldest_newest_stamp_vector.begin(),
          [](const ImuSyncBufferUnsync::Ptr imu_buffer) {
            return imu_buffer->getOldestAndNewestStamp();
          });

    // imu buffers are not consistent with the image buffers
    if (!validateImuBuffers(
          sync_imgs_ready_to_process_stamp_,
          sync_imgs_ready_to_process_stamp_,
          oldest_newest_stamp_vector))
    {
      return;
    }

    // If this is the very first image bundle, we send all IMU messages that we have
    // received so far. For every later image bundle, we just send the IMU messages
    // that we have received in between.
    for (size_t i = 0; i < num_imus_; ++i)
    {
      if(last_img_bundle_min_stamp_ < 0)
      {
        int64_t oldest_stamp = std::get<0>(oldest_newest_stamp_vector[i]);
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i]->getBetweenValuesInterpolated(
              oldest_stamp,
              sync_imgs_ready_to_process_stamp_);
      }
      else
      {
        std::tie(imu_timestamps[i], imu_measurements[i]) =
            imu_buffers_[i]->getBetweenValuesInterpolated(
              last_img_bundle_min_stamp_,
              sync_imgs_ready_to_process_stamp_);
      }
    }
  }

  // Let's process the callback.
  cam_imu_callback_(sync_imgs_ready_to_process_, imu_timestamps, imu_measurements);

  // Reset Buffer:
  for (size_t i = 0; i <= img_buffer_.size(); ++i)
  {
    ImageBufferItem& item = img_buffer_[i];
    if (std::abs(sync_imgs_ready_to_process_stamp_ - item.stamp)
        < c_camera_bundle_time_accuracy_ns)
    {
      item.reset();
    }
  }
  last_img_bundle_min_stamp_ = sync_imgs_ready_to_process_stamp_;
  sync_imgs_ready_to_process_stamp_ = -1;
  sync_imgs_ready_to_process_.clear();
}

} // namespace ze
