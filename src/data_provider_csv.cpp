#include <ze/data_provider/data_provider_csv.h>

#include <fstream>
#include <iostream>
#include <glog/logging.h>

#include <ze/common/time.h>
#include <ze/common/string_utils.h>
#include <imp/bridge/opencv/cv_bridge.hpp>

namespace ze {

namespace dataset {

ImageBase::Ptr CameraMeasurement::getImage() const
{
  ImageCv8uC1::Ptr img;
  cvBridgeLoad<Pixel8uC1, PixelType::i8uC1>(
        img, image_path_filename, PixelOrder::gray);
  CHECK_NOTNULL(img.get());
  CHECK(img->numel() > 0);
  return img;
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
    const std::string& imu_topic,
    const std::map<std::string, size_t>& camera_topics)
  : DataProviderBase(DataProviderType::Csv)
{
  VLOG(1) << "Loading .csv dataset from directory \"" << csv_directory << "\".";

  loadImuData(csv_directory + ensureLeftSlash(imu_topic), 0u);

  for(auto it : camera_topics)
  {
    std::string dir = csv_directory + ensureLeftSlash(it.first);
    loadCameraData(dir, it.second, millisecToNanosec(100));
  }

  buffer_it_ = buffer_.cbegin();
  VLOG(1) << "done.";
}

void DataProviderCsv::spin()
{
  while(ok())
    spinOnce();
}

bool DataProviderCsv::spinOnce()
{
  if(buffer_it_ != buffer_.cend())
  {
    const dataset::MeasurementBase::Ptr& data = buffer_it_->second;
    switch (data->type)
    {
      case dataset::MeasurementType::Camera:
      {
        if(camera_callback_)
        {
          dataset::CameraMeasurement::ConstPtr cam_data =
              std::dynamic_pointer_cast<const dataset::CameraMeasurement>(data);

          ImageBase::Ptr img = cam_data->getImage();
          camera_callback_(cam_data->stamp_ns, img, cam_data->camera_index);
        }
        else
        {
          LOG_FIRST_N(WARNING, 1) << "No camera callback registered but measurements available.";
        }
        break;
      }
      case dataset::MeasurementType::Imu:
      {
        if(imu_callback_)
        {
          dataset::ImuMeasurement::ConstPtr imu_data =
                std::dynamic_pointer_cast<const dataset::ImuMeasurement>(data);
          imu_callback_(imu_data->stamp_ns, imu_data->acc, imu_data->gyr);
        }
        else
        {
          LOG_FIRST_N(WARNING, 1) << "No IMU callback registered but measurements available";
        }
        break;
      }
      default:
        LOG(FATAL) << "Unhandled message type: " << static_cast<int>(data->type);
        break;
    }
    ++buffer_it_;
    return true;
  }
  return false;
}

bool DataProviderCsv::ok() const
{
  return buffer_it_ != buffer_.cend() && !shutdown_;
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
    std::vector<std::string> items = splitString(line, ',');
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
      std::vector<std::string> items = splitString(line, ',');
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

} // namespace ze
