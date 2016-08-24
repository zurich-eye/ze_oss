// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

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
  , uses_split_messages_(false)
{
  VLOG(1) << "Create Dataprovider for synchronized Gyro/Accel";

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

DataProviderRostopic::DataProviderRostopic(
    const std::map<std::string, size_t>& accel_topic_imuidx_map,
    const std::map<std::string, size_t>& gyro_topic_imuidx_map,
    const std::map<std::string, size_t>& img_topic_camidx_map,
    uint32_t polling_rate,
    uint32_t img_queue_size,
    uint32_t imu_queue_size)
  : DataProviderBase(DataProviderType::Rostopic)
  , img_transport_(nh_)
  , polling_rate_(polling_rate)
  , uses_split_messages_(true)
{
  VLOG(1) << "Create Dataprovider for UN-synchronized Gyro/Accel";

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

  for (auto it : accel_topic_imuidx_map)
  {
    auto cb = std::bind(&DataProviderRostopic::accelCallback,
                          this, std::placeholders::_1, it.second);
    sub_accels_.emplace_back(
          nh_.subscribe<ze_ros_msg::bmx055_acc>(it.first, imu_queue_size, cb));
    VLOG(1) << "Subscribed to accel topic " << it.first;
  }

  for (auto it : gyro_topic_imuidx_map)
  {
    auto cb = std::bind(&DataProviderRostopic::gyroCallback,
                          this, std::placeholders::_1, it.second);
    sub_gyros_.emplace_back(
          nh_.subscribe<ze_ros_msg::bmx055_gyr>(it.first, imu_queue_size, cb));
    VLOG(1) << "Subscribed to gyro topic " << it.first;
  }
}

size_t DataProviderRostopic::cameraCount() const
{
  return sub_cams_.size();
}

size_t DataProviderRostopic::imuCount() const
{
  if (uses_split_messages_)
  {
    return sub_accels_.size();
  }

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
    VLOG(1) << "Data Provider was paused/terminated.";
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

void DataProviderRostopic::accelCallback(
    const ze_ros_msg::bmx055_accConstPtr& m_acc,
    uint32_t imu_idx)
{
  if (!accel_callback_)
  {
    LOG_FIRST_N(WARNING, 1) << "No Accel callback registered but measurements available";
    return;
  }

  const Vector3 acc(
        m_acc->linear_acceleration.x,
        m_acc->linear_acceleration.y,
        m_acc->linear_acceleration.z);
  int64_t stamp = m_acc->header.stamp.toNSec();
  accel_callback_(stamp, acc, imu_idx);
}

void DataProviderRostopic::gyroCallback(
    const ze_ros_msg::bmx055_gyrConstPtr& m_gyr,
    uint32_t imu_idx)
{
  if (!gyro_callback_)
  {
    LOG_FIRST_N(WARNING, 1) << "No Gyro callback registered but measurements available";
    return;
  }

  const Vector3 gyr(
        m_gyr->angular_velocity.x,
        m_gyr->angular_velocity.y,
        m_gyr->angular_velocity.z);

  int64_t stamp = m_gyr->header.stamp.toNSec();
  gyro_callback_(stamp, gyr, imu_idx);
}


} // namespace ze
