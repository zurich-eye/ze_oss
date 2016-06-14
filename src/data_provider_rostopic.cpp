#include <ze/data_provider/data_provider_rostopic.hpp>

#include <imp/bridge/ros/ros_bridge.hpp>
#include <ze/common/logging.hpp>
#include <ze/common/time_conversions.h>
#include <ze/common/string_utils.h>
#include <ze/common/path_utils.h>

namespace ze {

DataProviderRostopic::DataProviderRostopic(
    const std::map<std::string, size_t>& imu_topic_imuidx_map,
    const std::map<std::string, size_t>& img_topic_camidx_map,
    uint32_t polling_rate,
    uint32_t img_queue_size,
    uint32_t imu_queue_size)
  : DataProviderBase(DataProviderType::Rostopic)
  , img_transport_(nh_)
  , polling_rate_(polling_rate)
{
  nh_.setCallbackQueue(&queue_);
  img_transport_ = image_transport::ImageTransport(nh_);

  // Subscribe to camera:
  for (auto it : img_topic_camidx_map)
  {
    auto cb = std::bind(&DataProviderRostopic::imgCallback,
                        this, std::placeholders::_1, it.second);
    sub_cams_.emplace_back(img_transport_.subscribe(it.first, img_queue_size, cb));
    VLOG(1) << "Subscribed to camera topic " << it.first;
  }

  for (auto it : imu_topic_imuidx_map)
  {
    auto cb = std::bind(&DataProviderRostopic::imuCallback,
                          this, std::placeholders::_1, it.second);
    sub_imus_.emplace_back(
          nh_.subscribe<sensor_msgs::Imu>(it.first, imu_queue_size, cb));
    VLOG(1) << "Subscribed to imu topic " << it.first;
  }
}

size_t DataProviderRostopic::cameraCount() const
{
  return sub_cams_.size();
}

size_t DataProviderRostopic::imuCount() const
{
  return sub_imus_.size();
}

bool DataProviderRostopic::spinOnce()
{
  queue_.callAvailable(ros::WallDuration(ros::Rate(polling_rate_)));
  return ok();
}

bool DataProviderRostopic::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was terminated.";
    return false;
  }
  if (!ros::ok())
  {
    VLOG(1) << "ROS not OK.";
    return false;
  }
  return true;
}

void DataProviderRostopic::imgCallback(
    const sensor_msgs::ImageConstPtr& m_img,
    uint32_t cam_idx)
{
  if (!camera_callback_)
  {
    LOG_FIRST_N(WARNING, 1) << "No Image callback registered but measurements available";
    return;
  }

  ze::ImageBase::Ptr img = toImageCpu(*m_img);
  camera_callback_(m_img->header.stamp.toNSec(), img, cam_idx);
}

void DataProviderRostopic::imuCallback(
    const sensor_msgs::ImuConstPtr& m_imu,
    uint32_t imu_idx)
{
  if (!imu_callback_)
  {
    LOG_FIRST_N(WARNING, 1) << "No IMU callback registered but measurements available";
    return;
  }

  const Vector3 gyr(
        m_imu->angular_velocity.x,
        m_imu->angular_velocity.y,
        m_imu->angular_velocity.z);
  const Vector3 acc(
        m_imu->linear_acceleration.x,
        m_imu->linear_acceleration.y,
        m_imu->linear_acceleration.z);
  int64_t stamp = m_imu->header.stamp.toNSec();
  imu_callback_(stamp, acc, gyr, imu_idx);
}

} // namespace ze
