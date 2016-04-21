#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <ze/data_provider/data_provider_base.h>

namespace ze {

class DataProviderRosbag : public DataProviderBase
{
public:

  DataProviderRosbag(
      const std::string& bag_filename,
      const std::string& imu_topic,
      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderRosbag() = default;

  virtual void spin() override;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  size_t size() const;

private:

  std::unique_ptr<rosbag::Bag> bag_;
  std::unique_ptr<rosbag::View> bag_view_;
  rosbag::View::iterator bag_view_it_;
  int n_processed_images_ = 0;
  bool quit_ = false;

  // subscribed topics:
  std::map<std::string, size_t> img_topic_camidx_map_; // camera_topic --> camera_id
  std::string imu_topic_;
  int64_t last_imu_stamp_ = -1;
};

} // namespace ze
