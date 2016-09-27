// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <ze/data_provider/camera_imu_synchronizer_unsync.hpp>

#include <functional>
#include <gflags/gflags.h>

#include <ze/common/logging.hpp>
#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

CameraImuSynchronizerUnsync::CameraImuSynchronizerUnsync(
    DataProviderBase& data_provider,
    const std::vector<ImuModel::Ptr>& imu_models)
  : CameraImuSynchronizerBase(data_provider)
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
    imu_buffers_.push_back(std::make_shared<ImuSyncBuffer>(imu_model));
  }
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
          [](const ImuSyncBuffer::Ptr imu_buffer) {
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
