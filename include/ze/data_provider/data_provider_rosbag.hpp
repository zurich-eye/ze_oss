#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

class DataProviderRosbag : public DataProviderBase
{
public:
  // optional: imu_topics, required: camera_topics
  DataProviderRosbag(
      const std::string& bag_filename,
      const std::map<std::string, size_t>& imu_topics,
      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderRosbag() = default;

  virtual void spin() override;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imuCount() const;
  virtual size_t cameraCount() const;

  size_t size() const;

private:
  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
  rosbag::View::iterator bag_view_it_;
  int n_processed_images_ = 0;

  // subscribed topics:
  std::map<std::string, size_t> img_topic_camidx_map_; // camera_topic --> camera_id
  std::map<std::string, size_t> imu_topic_imuidx_map_; // imu_topic --> imu_id
  int64_t last_imu_stamp_ = -1;
};

} // namespace ze
