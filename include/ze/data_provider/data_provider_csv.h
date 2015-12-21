#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/types.h>
#include <ze/common/macros.h>
#include <ze/data_provider/data_provider_base.h>

// fwd
namespace cv {
class Mat;
}

namespace ze {
namespace dataset {

enum class MeasurementType {
  kImu,
  kCamera,
  kFeatureTrack,
};

struct MeasurementBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(MeasurementBase);

  MeasurementBase() = delete;
  MeasurementBase(int64_t stamp_ns, MeasurementType type)
    : stamp_ns(stamp_ns)
    , type(type)
  {}
  virtual ~MeasurementBase() = default;

  const int64_t stamp_ns;
  const MeasurementType type;
};

struct ImuMeasurement : public MeasurementBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(ImuMeasurement);

  ImuMeasurement() = delete;
  ImuMeasurement(int64_t stamp_ns, const Eigen::Vector3d& acc, const Eigen::Vector3d& gyr)
    : MeasurementBase(stamp_ns, MeasurementType::kImu)
    , acc(acc)
    , gyr(gyr)
  {}
  virtual ~ImuMeasurement() = default;

  const Eigen::Vector3d acc;
  const Eigen::Vector3d gyr;
};

struct CameraMeasurement : public MeasurementBase
{
  ZE_POINTER_TYPEDEFS(CameraMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  CameraMeasurement() = delete;
  CameraMeasurement(int64_t stamp_ns, size_t cam_idx, const std::string& img_path)
    : MeasurementBase(stamp_ns, MeasurementType::kCamera)
    , camera_index(cam_idx)
    , image_path_filename(img_path)
  {}
  virtual ~CameraMeasurement() = default;

  void getImage(cv::Mat* image) const;

  const size_t camera_index;
  const std::string image_path_filename;
};

struct FeatureTrackMeasurement : public MeasurementBase
{
  ZE_POINTER_TYPEDEFS(FeatureTrackMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureTrackMeasurement() = delete;
  FeatureTrackMeasurement(int64_t stamp_ns, size_t cam_idx, int track_id,
                          const Eigen::Vector2d& keypoint_measurement,
                          double keypoint_std_dev)
    : MeasurementBase(stamp_ns, MeasurementType::kFeatureTrack)
    , camera_index(cam_idx)
    , track_id(track_id)
    , keypoint_measurement(keypoint_measurement)
    , keypoint_std_dev(keypoint_std_dev)
  {}
  virtual ~FeatureTrackMeasurement() = default;

  const size_t camera_index;
  const int track_id;
  const Eigen::Vector2d keypoint_measurement;
  const double keypoint_std_dev;
};

} // namespace dataset

class DataProviderCsv : public DataProviderBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using StampMeasurementPair = std::pair<int64_t, std::shared_ptr<dataset::MeasurementBase>> ;
  using DataBuffer = std::multimap<int64_t, std::shared_ptr<dataset::MeasurementBase>> ;

  DataProviderCsv(
      const std::string& csv_directory,
      const std::vector<size_t> imu_indices,
      const std::vector<size_t> camera_indices,
      const std::vector<size_t> track_indices);

  virtual ~DataProviderCsv() = default;

  // Read next data field and process callback. Waits until callback is processed.
  virtual void spinOnceBlocking();

private:

  void loadImuData(
      const std::string data_dir,
      const int64_t playback_delay);

  void loadCameraData(
      const std::string& data_dir,
      const size_t camera_index,
      int64_t playback_delay);

  void loadFeatureTracksData(
      const std::string& data_dir,
      const size_t camera_index,
      int64_t playback_delay);

  // Buffer to chronologically sort the data.
  DataBuffer buffer_;
};

} // namespace ze
