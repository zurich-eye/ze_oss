#include <ze/data_provider/data_provider_factory.hpp>
#include <ze/data_provider/data_provider_base.hpp>
#include <ze/data_provider/data_provider_csv.hpp>
#include <ze/data_provider/data_provider_rosbag.hpp>
#include <ze/data_provider/data_provider_rostopic.hpp>

DEFINE_string(bag_filename, "dataset.bag", "Name of bagfile in data_dir.");

DEFINE_string(topic_cam0, "/cam0/image_raw", "");
DEFINE_string(topic_cam1, "/cam1/image_raw", "");
DEFINE_string(topic_cam2, "/cam2/image_raw", "");
DEFINE_string(topic_cam3, "/cam2/image_raw", "");

DEFINE_string(topic_imu0, "/imu0", "");
DEFINE_string(topic_imu1, "/imu1", "");
DEFINE_string(topic_imu2, "/imu2", "");
DEFINE_string(topic_imu3, "/imu3", "");

DEFINE_string(topic_acc0, "/acc0", "");
DEFINE_string(topic_acc1, "/acc1", "");
DEFINE_string(topic_acc2, "/acc2", "");
DEFINE_string(topic_acc3, "/acc3", "");

DEFINE_string(topic_gyr0, "/gyr0", "");
DEFINE_string(topic_gyr1, "/gyr1", "");
DEFINE_string(topic_gyr2, "/gyr2", "");
DEFINE_string(topic_gyr3, "/gyr3", "");

DEFINE_int32(data_source, 1, " 0: CSV, 1: Rosbag, 2: Rostopic");
DEFINE_string(data_dir, "", "Directory for csv dataset.");
DEFINE_uint64(num_imus, 1, "Number of IMUs used in the pipeline.");
DEFINE_uint64(num_accels, 1, "Number of Accelerometers used in the pipeline.");
DEFINE_uint64(num_gyros, 1, "Number of Gyroscopes used in the pipeline.");

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(const uint32_t num_cams)
{
  CHECK_GT(num_cams, 0u);
  CHECK_LE(num_cams, 4u);
  CHECK_LE(FLAGS_num_imus, 4u);

  // Fill camera topics.
  std::map<std::string, size_t> cam_topics;
  if (num_cams >= 1) cam_topics[FLAGS_topic_cam0] = 0;
  if (num_cams >= 2) cam_topics[FLAGS_topic_cam1] = 1;
  if (num_cams >= 3) cam_topics[FLAGS_topic_cam2] = 2;
  if (num_cams >= 4) cam_topics[FLAGS_topic_cam3] = 3;

  // Fill imu topics.
  std::map<std::string, size_t> imu_topics;
  if (FLAGS_num_imus >= 1) imu_topics[FLAGS_topic_imu0] = 0;
  if (FLAGS_num_imus >= 2) imu_topics[FLAGS_topic_imu1] = 1;
  if (FLAGS_num_imus >= 3) imu_topics[FLAGS_topic_imu2] = 2;
  if (FLAGS_num_imus >= 4) imu_topics[FLAGS_topic_imu3] = 3;

  // Fill accelerometer topics.
  std::map<std::string, size_t> acc_topics;
  if (FLAGS_num_accels >= 1) acc_topics[FLAGS_topic_acc0] = 0;
  if (FLAGS_num_accels >= 2) acc_topics[FLAGS_topic_acc1] = 1;
  if (FLAGS_num_accels >= 3) acc_topics[FLAGS_topic_acc2] = 2;
  if (FLAGS_num_accels >= 4) acc_topics[FLAGS_topic_acc3] = 3;

  // Fill gyroscope topics.
  std::map<std::string, size_t> gyr_topics;
  if (FLAGS_num_gyros >= 1) gyr_topics[FLAGS_topic_gyr0] = 0;
  if (FLAGS_num_gyros >= 2) gyr_topics[FLAGS_topic_gyr1] = 1;
  if (FLAGS_num_gyros >= 3) gyr_topics[FLAGS_topic_gyr2] = 2;
  if (FLAGS_num_gyros >= 4) gyr_topics[FLAGS_topic_gyr3] = 3;

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
      // Use the split imu dataprovider
      if (FLAGS_num_accels != 0 && FLAGS_num_gyros != 0)
      {
        data_provider.reset(new DataProviderRosbag(FLAGS_bag_filename,
                                                   acc_topics,
                                                   gyr_topics,
                                                   cam_topics));
      }
      else
      {
        data_provider.reset(new DataProviderRosbag(FLAGS_bag_filename,
                                                   imu_topics,
                                                   cam_topics));
      }
      break;
    }
    case 2: // Rostopic
    {
      data_provider.reset(new DataProviderRostopic(imu_topics,
                                                   cam_topics));

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
