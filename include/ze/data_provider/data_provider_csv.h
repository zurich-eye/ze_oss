#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ze/common/types.h>
#include <ze/common/macros.h>
#include <ze/data_provider/data_provider_base.h>
#include <imp/core/image_base.hpp>

// fwd
namespace cv {
class Mat;
}

namespace ze {
namespace dataset {

enum class MeasurementType {
  Imu,
  Camera,
  FeatureTrack,
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
    : MeasurementBase(stamp_ns, MeasurementType::Imu)
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
    : MeasurementBase(stamp_ns, MeasurementType::Camera)
    , camera_index(cam_idx)
    , image_filename(img_path)
  {}
  virtual ~CameraMeasurement() = default;

  ImageBase::Ptr loadImage() const;

  const size_t camera_index;
  const std::string image_filename;
};

struct FeatureTrackMeasurement : public MeasurementBase
{
  ZE_POINTER_TYPEDEFS(FeatureTrackMeasurement);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FeatureTrackMeasurement() = delete;
  FeatureTrackMeasurement(int64_t stamp_ns, size_t cam_idx, int track_id,
                          const Eigen::Vector2d& keypoint_measurement,
                          double keypoint_std_dev)
    : MeasurementBase(stamp_ns, MeasurementType::FeatureTrack)
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
      const std::string& imu_topic,
      const std::map<std::string, size_t>& camera_topics);

  virtual ~DataProviderCsv() = default;

  virtual void spin() override;

  virtual bool spinOnce() override;

  virtual bool ok() const override;

  virtual size_t imu_count() const;
  virtual size_t camera_count() const;

  inline size_t size() const
  {
    return buffer_.size();
  }

private:

  void loadImuData(
      const std::string data_dir,
      const int64_t playback_delay);

  void loadCameraData(
      const std::string& data_dir,
      const size_t camera_index,
      int64_t playback_delay);

  //! Buffer to chronologically sort the data.
  DataBuffer buffer_;

  //! Points to the next published buffer value. Buffer can't change once loaded!
  DataBuffer::const_iterator buffer_it_;

  std::map<std::string, size_t> camera_topics_;
};

} // namespace ze
