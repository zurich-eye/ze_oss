#include <ze/data_provider/data_provider_csv.h>

#include <fstream>
#include <iostream>
#include <ze/common/logging.hpp>

#include <ze/common/time_conversions.h>
#include <ze/common/string_utils.h>
#include <ze/common/file_utils.h>
#include <imp/bridge/opencv/cv_bridge.hpp>

namespace ze {

namespace dataset {

ImageBase::Ptr CameraMeasurement::loadImage() const
{
  //! @todo: Make an option which pixel-type to load.
  ImageCv8uC1::Ptr img;
  cvBridgeLoad<Pixel8uC1>(img, image_filename, PixelOrder::gray);
  CHECK_NOTNULL(img.get());
  CHECK(img->numel() > 0);
  return img;
}

} // namespace dataset

DataProviderCsv::DataProviderCsv(
    const std::string& csv_directory,
    const std::string& imu_topic,
    const std::map<std::string, size_t>& camera_topics)
  : DataProviderBase(DataProviderType::Csv)
  , camera_topics_(camera_topics)
{
  VLOG(1) << "Loading .csv dataset from directory \"" << csv_directory << "\".";

  if (!imu_topic.empty())
  {
    loadImuData(joinPath(csv_directory, imu_topic), 0u);
  }

  for (auto it : camera_topics)
  {
    std::string dir = joinPath(csv_directory, it.first);
    loadCameraData(dir, it.second, millisecToNanosec(100));
  }

  buffer_it_ = buffer_.cbegin();
  VLOG(1) << "done.";
}

void DataProviderCsv::spin()
{
  while (ok())
  {
    spinOnce();
  }
}

bool DataProviderCsv::spinOnce()
{
  if (buffer_it_ != buffer_.cend())
  {
    const dataset::MeasurementBase::Ptr& data = buffer_it_->second;
    switch (data->type)
    {
      case dataset::MeasurementType::Camera:
      {
        if (camera_callback_)
        {
          dataset::CameraMeasurement::ConstPtr cam_data =
              std::dynamic_pointer_cast<const dataset::CameraMeasurement>(data);

          ImageBase::Ptr img = cam_data->loadImage();
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
        if (imu_callback_)
        {
          dataset::ImuMeasurement::ConstPtr imu_data =
                std::dynamic_pointer_cast<const dataset::ImuMeasurement>(data);
          // csv datasets only support a single imu.
          imu_callback_(imu_data->stamp_ns, imu_data->acc, imu_data->gyr, 0);
        }
        else
        {
          LOG_FIRST_N(WARNING, 1) << "No IMU callback registered but measurements available";
        }
        break;
      }
      default:
      {
        LOG(FATAL) << "Unhandled message type: " << static_cast<int>(data->type);
        break;
      }
    }
    ++buffer_it_;
    return true;
  }
  return false;
}

bool DataProviderCsv::ok() const
{
  if (!running_)
  {
    VLOG(1) << "Data Provider was terminated.";
    return false;
  }
  if (buffer_it_ == buffer_.cend())
  {
    VLOG(1) << "All data processed.";
    return false;
  }
  return true;
}

size_t DataProviderCsv::camera_count() const
{
  return camera_topics_.size();
}

size_t DataProviderCsv::imu_count() const
{
  // Only a single imu is supported in csv files.
  return 1;
}

void DataProviderCsv::loadImuData(const std::string data_dir, const int64_t playback_delay)
{
  const std::string kHeader = "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]";
  std::ifstream fs;
  openFileStreamAndCheckHeader(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while (std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 7u);
    Eigen::Vector3d acc, gyr;
    acc << std::stod(items[4]), std::stod(items[5]), std::stod(items[6]);
    gyr << std::stod(items[1]), std::stod(items[2]), std::stod(items[3]);
    auto imu_measurement =
        std::make_shared<dataset::ImuMeasurement>(std::stoll(items[0]), acc, gyr);

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
  openFileStreamAndCheckHeader(data_dir+"/data.csv", kHeader, &fs);
  std::string line;
  size_t i = 0;
  while (std::getline(fs, line))
  {
    std::vector<std::string> items = splitString(line, ',');
    CHECK_EQ(items.size(), 2u);
    auto camera_measurement =
        std::make_shared<dataset::CameraMeasurement>(
          std::stoll(items[0]), camera_index, data_dir + "/data/" + items[1]);

    buffer_.insert(std::make_pair(
                     camera_measurement->stamp_ns + playback_delay,
                     camera_measurement));
    ++i;
  }
  VLOG(2) << "Loaded " << i << " camera measurements.";
  fs.close();
}

} // namespace ze
