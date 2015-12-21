#include <ze/data_provider/data_provider_csv.h>

#include <fstream>
#include <iostream>
#include <glog/logging.h>
#include <opencv2/highgui/highgui.hpp>

#include <ze/common/time.h>
#include <ze/common/string_utils.h>

namespace ze {

namespace dataset {

void CameraMeasurement::getImage(cv::Mat* img) const
{
  CHECK_NOTNULL(img);
  *img = cv::imread(image_path_filename, CV_LOAD_IMAGE_GRAYSCALE);
  CHECK_NOTNULL(img->data);
}

} // namespace dataset

namespace utils {

void checkHeaderAndOpenStream(
    const std::string& filename,
    const std::string& header,
    std::ifstream* fs)
{
  CHECK_NOTNULL(fs);
  VLOG(1) << "Reading file " << filename;
  fs->open(filename.c_str(), std::ios::in);
  CHECK(fs);
  CHECK(fs->is_open()) << "Failed to open file " << filename;
  CHECK(!fs->eof()) << "File seems to contain no content!";

  std::string line;
  std::getline(*fs, line);
  CHECK_EQ(line, header) << "Invalid header.";
}

} // namespace utils

DataProviderCsv::DataProviderCsv(
    const std::string& csv_directory,
    const std::vector<size_t> imu_indices,
    const std::vector<size_t> camera_indices,
    const std::vector<size_t> track_indices)
  : DataProviderBase()
{
  VLOG(1) << "Loading csv dataset from directory \"" << csv_directory << "\".";

  CHECK_LE(imu_indices.size(), 1u) << "Using multiple IMUs not implemented";
  for(size_t i : imu_indices)
  {
    loadImuData(csv_directory + "/imu" + std::to_string(i), 0u);
  }

  for(size_t i : camera_indices)
  {
    std::string dir = csv_directory + "/cam" + std::to_string(i);
    loadCameraData(dir, i, ze::time::millisecToNanosec(100));
  }

  for(size_t i : track_indices)
  {
    std::string dir = csv_directory + "/tracks" + std::to_string(i);
    loadFeatureTracksData(dir, i, ze::time::millisecToNanosec(80));
  }

  VLOG(1) << "done.";
}

void DataProviderCsv::spinOnceBlocking()
{
  for(const DataProviderCsv::StampMeasurementPair& item : buffer_)
  {
    const dataset::MeasurementBase::Ptr& data = item.second;
    switch (data->type)
    {
      case dataset::MeasurementType::kCamera:
      {
        if(camera_callback_)
        {
          dataset::CameraMeasurement::ConstPtr cam_data =
              std::dynamic_pointer_cast<const dataset::CameraMeasurement>(data);

          cv::Mat img;
          cam_data->getImage(&img);

          camera_callback_(cam_data->stamp_ns, img, cam_data->camera_index);
        }
        else
        {
          static bool warn_once = false;
          if(!warn_once)
          {
            LOG(WARNING) << "No camera callback registered but measurements available.";
            warn_once = true;
          }
        }
        break;
      }
      case dataset::MeasurementType::kImu:
      {
        if(imu_callback_)
        {
          dataset::ImuMeasurement::ConstPtr imu_data =
                std::dynamic_pointer_cast<const dataset::ImuMeasurement>(data);
          imu_callback_(imu_data->stamp_ns, imu_data->acc, imu_data->gyr);
        }
        else
        {
          static bool warn_once = false;
          if(!warn_once)
          {
            LOG(WARNING) << "No IMU callback registered but measurements available";
            warn_once = true;
          }
        }
        break;
      }
      default:
        LOG(FATAL) << "Unhandled message type: " << static_cast<int>(data->type);
        break;
    }
  }
}

void DataProviderCsv::loadImuData(const std::string data_dir, const int64_t playback_delay)
{
  const std::string kHeader = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]";
  std::ifstream fs;
  utils::checkHeaderAndOpenStream(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while(std::getline(fs, line))
  {
    std::vector<std::string> items = ze::common::splitString(line, ',');
    CHECK_EQ(items.size(), 7u);
    Eigen::Vector3d acc, gyr;
    acc << std::stod(items[4]), std::stod(items[5]), std::stod(items[6]);
    gyr << std::stod(items[1]), std::stod(items[2]), std::stod(items[3]);
    dataset::ImuMeasurement::Ptr imu_measurement(
        new dataset::ImuMeasurement(std::stoll(items[0]), acc, gyr));

    buffer_.insert(std::make_pair(
                     imu_measurement->stamp_ns + playback_delay,
                     imu_measurement));
    ++i;
  }
  VLOG(2) << "Loaded " << i << " IMU measurements.";
  fs.close();
}

void DataProviderCsv::loadCameraData(
    const std::string& data_dir,
    const size_t camera_index,
    int64_t playback_delay)
{
  const std::string kHeader = "#timestamp [ns],filename";
  std::ifstream fs;
  utils::checkHeaderAndOpenStream(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while(std::getline(fs, line))
  {
      std::vector<std::string> items = ze::common::splitString(line, ',');
      CHECK_EQ(items.size(), 2u);
    dataset::CameraMeasurement::Ptr camera_measurement(
        new dataset::CameraMeasurement(
            std::stoll(items[0]), camera_index, data_dir + "/data/" + items[1]));

    buffer_.insert(std::make_pair(
                     camera_measurement->stamp_ns + playback_delay,
                     camera_measurement));
    ++i;
  }
  VLOG(2) << "Loaded " << i << " camera measurements.";
  fs.close();
}

void DataProviderCsv::loadFeatureTracksData(
    const std::string& /*data_dir*/,
    const size_t /*camera_index*/,
    int64_t /*playback_delay*/)
{
  LOG(FATAL) << "Not implemented";
}

} // namespace ze
