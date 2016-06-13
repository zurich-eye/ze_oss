#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>

DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile in data_dir.");

DEFINE_string(topic_cam0, "/cam0/image_raw", "");
DEFINE_string(topic_cam1, "/cam1/image_raw", "");
DEFINE_string(topic_cam2, "/cam2/image_raw", "");
DEFINE_string(topic_cam3, "/cam2/image_raw", "");

DEFINE_string(topic_imu0, "/imu0", "");
DEFINE_string(topic_imu1, "/imu1", "");
DEFINE_string(topic_imu2, "/imu2", "");
DEFINE_string(topic_imu3, "/imu3", "");

DEFINE_int32(data_source, 1, " 0: CSV, 1: Rosbag");
DEFINE_string(data_dir, "", "Directory for csv dataset.");

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(
    const uint32_t num_cams, const uint32_t num_imus)
{
  CHECK_GT(num_cams, 0u);
  CHECK_LE(num_cams, 4u);
  CHECK_GT(num_imus, 0u);
  CHECK_LE(num_cams, 4u);

  // Fill camera topics.
  std::map<std::string, size_t> cam_topics;
  if (num_cams >= 1) cam_topics[FLAGS_topic_cam0] = 0;
  if (num_cams >= 2) cam_topics[FLAGS_topic_cam1] = 1;
  if (num_cams >= 3) cam_topics[FLAGS_topic_cam2] = 2;
  if (num_cams >= 4) cam_topics[FLAGS_topic_cam3] = 3;

  // Fill imu topics.
  std::map<std::string, size_t> imu_topics;
  if (num_imus >= 1) imu_topics[FLAGS_topic_imu0] = 0;
  if (num_imus >= 2) imu_topics[FLAGS_topic_imu1] = 1;
  if (num_imus >= 3) imu_topics[FLAGS_topic_imu2] = 2;
  if (num_imus >= 4) imu_topics[FLAGS_topic_imu3] = 3;

  // Create data provider.
  ze::DataProviderBase::Ptr data_provider;
  switch (FLAGS_data_source)
  {
    case 0: // CSV
    {
      data_provider.reset(
            new DataProviderCsv(FLAGS_data_dir, imu_topics, cam_topics));
      break;
    }
    case 1: // Rosbag
    {
      data_provider.reset(
            new DataProviderRosbag(FLAGS_bag_filename, imu_topics, cam_topics));
      break;
    }
    default:
    {
      LOG(FATAL) << "Data source not known.";
      break;
    }
  }

  return data_provider;
}

} // namespace ze
