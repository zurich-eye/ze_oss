#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>

#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

class DataProviderRostopic : public DataProviderBase
{
public:
  // optional: imu_topics, required: camera_topics
  DataProviderRostopic(
      const std::map<std::string, size_t>& imu_topics,
      const std::map<std::string, size_t>& camera_topics,
      uint32_t polling_rate = 1000u,
      uint32_t img_queue_size = 100u,
      uint32_t imu_queue_size = 1000u);

  virtual ~DataProviderRostopic() = default;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imuCount() const;

  virtual size_t cameraCount() const;

  void imgCallback(
      const sensor_msgs::ImageConstPtr& m_img,
      uint32_t cam_idx);

  void imuCallback(
      const sensor_msgs::ImuConstPtr& m_imu,
      uint32_t imu_idx);

private:
  ros::CallbackQueue queue_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport img_transport_;
  std::vector<image_transport::Subscriber> sub_cams_;
  std::vector<ros::Subscriber> sub_imus_;
  uint32_t polling_rate_;
};

} // namespace ze
