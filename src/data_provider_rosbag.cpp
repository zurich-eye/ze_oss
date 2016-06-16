#include <ze/data_provider/data_provider_rosbag.hpp>

#include <ze/common/logging.hpp>
#include <rosbag/query.h>

#include <ze/common/time_conversions.h>
#include <ze/common/string_utils.h>
#include <ze/common/path_utils.h>

#include <imp/bridge/ros/ros_bridge.hpp>

DEFINE_int32(data_source_stop_after_n_frames, -1,
             "How many frames should be processed?");

namespace ze {

DataProviderRosbag::DataProviderRosbag(
    const std::string& bag_filename,
    const std::map<std::string, size_t>& imu_topic_imuidx_map,
    const std::map<std::string, size_t>& img_topic_camidx_map)
  : DataProviderBase(DataProviderType::Rosbag)
  , img_topic_camidx_map_(img_topic_camidx_map)
  , imu_topic_imuidx_map_(imu_topic_imuidx_map)
  , uses_split_messages_(false)
{
  //! @todo: Check if topics exists.
  //! @todo: Display number of messages per topic in the beginning.

  loadRosbag(bag_filename);

  // empty bracket initialzer calls default constructor and
  // adds an empty element to the map
  if (imu_topic_imuidx_map_.size() == 1 &&
      imu_topic_imuidx_map_.begin()->first.empty())
  {
    imu_topic_imuidx_map_.clear();
  }

  std::vector<std::string> topics;
  for (auto it : img_topic_camidx_map_)
  {
    topics.push_back(it.first);
  }
  for (auto it : imu_topic_imuidx_map_)
  {
    topics.push_back(it.first);
  }

  initBagView(topics);
}

DataProviderRosbag::DataProviderRosbag(
    const std::string& bag_filename,
    const std::map<std::string, size_t>& accel_topic_imuidx_map,
    const std::map<std::string, size_t>& gyro_topic_imuidx_map,
    const std::map<std::string, size_t>& img_topic_camidx_map)
  : DataProviderBase(DataProviderType::Rosbag)
  , img_topic_camidx_map_(img_topic_camidx_map)
  , accel_topic_imuidx_map_(accel_topic_imuidx_map)
  , gyro_topic_imuidx_map_(gyro_topic_imuidx_map)
  , uses_split_messages_(true)
{
  //! @todo: Check if topics exists.
  //! @todo: Display number of messages per topic in the beginning.

  CHECK_EQ(accel_topic_imuidx_map_.size(), gyro_topic_imuidx_map_.size());

  loadRosbag(bag_filename);

  // empty bracket initialzer calls default constructor and
  // adds an empty element to the map
  if (accel_topic_imuidx_map_.size() == 1 &&
      accel_topic_imuidx_map_.begin()->first.empty())
  {
    accel_topic_imuidx_map_.clear();
    gyro_topic_imuidx_map_.clear();
  }

  std::vector<std::string> topics;
  for (auto it : img_topic_camidx_map_)
  {
    topics.push_back(it.first);
  }
  for (auto it : accel_topic_imuidx_map_)
  {
    topics.push_back(it.first);
  }
  for (auto it : gyro_topic_imuidx_map_)
  {
    topics.push_back(it.first);
  }

  initBagView(topics);
}

void DataProviderRosbag::loadRosbag(const std::string& bag_filename)
{
  CHECK(fileExists(bag_filename)) << "File does not exist: " << bag_filename;
  VLOG(1) << "Opening rosbag: " << bag_filename << " ...";
  bag_.reset(new rosbag::Bag);
  try
  {
    bag_->open(bag_filename, rosbag::bagmode::Read);
  }
  catch (const std::exception e)
  {
    LOG(FATAL) << "Could not open rosbag " << bag_filename << ": " << e.what();
  }
}

void DataProviderRosbag::initBagView(const std::vector<std::string>& topics)
{
  bag_view_.reset(new rosbag::View(*bag_, rosbag::TopicQuery(topics)));
  bag_view_it_ = bag_view_->begin();

  // Ensure that topics exist
  // The connection info only contains topics that are available in the bag
  // If a topic is requested that is not avaiable, it does not show up in the info.
  std::vector<const rosbag::ConnectionInfo*> connection_infos =
      bag_view_->getConnections();
  CHECK(topics.size() == connection_infos.size())
      << "Not all requested topics founds in bagfile";
}

size_t DataProviderRosbag::cameraCount() const
{
  return img_topic_camidx_map_.size();
}

size_t DataProviderRosbag::imuCount() const
{
  if (uses_split_messages_)
  {
    return accel_topic_imuidx_map_.size();
  }

  return imu_topic_imuidx_map_.size();
}

bool DataProviderRosbag::spinOnce()
{
  if (bag_view_it_ != bag_view_->end())
  {
    const rosbag::MessageInstance m = *bag_view_it_;

    // Camera Messages:
    const sensor_msgs::ImageConstPtr m_img = m.instantiate<sensor_msgs::Image>();
    if (m_img && camera_callback_)
    {
      if (!cameraSpin(m_img, m))
      {
        return false;
      }
    }
    else if (m_img && !camera_callback_)
    {
      LOG_FIRST_N(WARNING, 1) << "No camera callback registered but measurements available";
    }

    // Imu Messages:
    const sensor_msgs::ImuConstPtr m_imu = m.instantiate<sensor_msgs::Imu>();
    if (m_imu && imu_callback_)
    {
      if (!imuSpin(m_imu, m))
      {
        return false;
      }
    }
    else if (m_imu && !imu_callback_)
    {
      LOG_FIRST_N(WARNING, 1) << "No IMU callback registered but measurements available";
    }

    // Accelerometer Messages:
    const ze_eval_node::bmx055_accConstPtr m_acc =
        m.instantiate<ze_eval_node::bmx055_acc>();
    if (m_acc && accel_callback_)
    {
      if (!accelSpin(m_acc, m))
      {
        return false;
      }
    }
    else if (m_acc && !accel_callback_)
    {
      LOG_FIRST_N(WARNING, 1) << "No Accelerometer callback registered but"
                              << "measurements available";
    }

    // Gyroscope Messages:
    const ze_eval_node::bmx055_gyrConstPtr m_gyr =
        m.instantiate<ze_eval_node::bmx055_gyr>();
    if (m_gyr && gyro_callback_)
    {
      if(!gyroSpin(m_gyr, m))
      {
        return false;
      }
    }
    else if (m_gyr && !gyro_callback_)
    {
      LOG_FIRST_N(WARNING, 1) << "No Gyroscope callback registered but"
                              << "measurements available";
    }

    ++bag_view_it_;
    return true;
  }
  return false;
}

bool DataProviderRosbag::cameraSpin(sensor_msgs::ImageConstPtr m_img,
                                    const rosbag::MessageInstance& m)
{
  auto it = img_topic_camidx_map_.find(m.getTopic());
  if (it != img_topic_camidx_map_.end())
  {
    ++n_processed_images_;
    if (FLAGS_data_source_stop_after_n_frames > 0
        && n_processed_images_ > FLAGS_data_source_stop_after_n_frames)
    {
      LOG(WARNING) << "Data source has reached max number of desired frames.";
      running_ = false;
      return false;
    }

    ze::ImageBase::Ptr img = toImageCpu(*m_img);
    camera_callback_(m_img->header.stamp.toNSec(), img, it->second);
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "Topic in bag that is not subscribed: " << m.getTopic();
  }

  return true;
}

bool DataProviderRosbag::imuSpin(sensor_msgs::ImuConstPtr m_imu,
                                 const rosbag::MessageInstance& m)
{
  auto it = imu_topic_imuidx_map_.find(m.getTopic());
  if (it != imu_topic_imuidx_map_.end())
  {
    const Vector3 gyr(
          m_imu->angular_velocity.x,
          m_imu->angular_velocity.y,
          m_imu->angular_velocity.z);
    const Vector3 acc(
          m_imu->linear_acceleration.x,
          m_imu->linear_acceleration.y,
          m_imu->linear_acceleration.z);
    int64_t stamp = m_imu->header.stamp.toNSec();
    CHECK_GT(stamp, last_imu_stamp_);
    imu_callback_(stamp, acc, gyr, it->second);
    last_imu_stamp_ = stamp;
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "Topic in bag that is not subscribed: " << m.getTopic();
  }

  return true;
}

bool DataProviderRosbag::accelSpin(ze_eval_node::bmx055_accConstPtr m_acc,
                                   const rosbag::MessageInstance& m)
{
  auto it = accel_topic_imuidx_map_.find(m.getTopic());
  if (it != accel_topic_imuidx_map_.end())
  {
    const Vector3 acc(
          m_acc->linear_acceleration.x,
          m_acc->linear_acceleration.y,
          m_acc->linear_acceleration.z);
    int64_t stamp = m_acc->header.stamp.toNSec();
    CHECK_GT(stamp, last_acc_stamp_);
    accel_callback_(stamp, acc, it->second);
    last_acc_stamp_ = stamp;
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "Topic in bag that is not subscribed: " << m.getTopic();
  }

  return true;
}

bool DataProviderRosbag::gyroSpin(ze_eval_node::bmx055_gyrConstPtr m_gyr,
                                  const rosbag::MessageInstance& m)
{
  auto it = gyro_topic_imuidx_map_.find(m.getTopic());
  if (it != gyro_topic_imuidx_map_.end())
  {
    const Vector3 gyr(
          m_gyr->angular_velocity.x,
          m_gyr->angular_velocity.y,
          m_gyr->angular_velocity.z);
    int64_t stamp = m_gyr->header.stamp.toNSec();
    CHECK_GT(stamp, last_gyr_stamp_);
    gyro_callback_(stamp, gyr, it->second);
    last_gyr_stamp_ = stamp;
  }
  else
  {
    LOG_FIRST_N(WARNING, 1) << "Topic in bag that is not subscribed: " << m.getTopic();
  }

  return true;
}

bool DataProviderRosbag::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was terminated.";
    return false;
  }
  if (bag_view_it_ == bag_view_->end())
  {
    VLOG(1) << "All data processed.";
    return false;
  }
  return true;
}

size_t DataProviderRosbag::size() const
{
  CHECK(bag_view_);
  return bag_view_->size();
}

} // namespace ze
