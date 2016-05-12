#pragma once

#include <string>

#include <ze/imu/accelerometer_model.h>
#include <ze/imu/gyroscope_model.h>
#include <ze/imu/imu_yaml_serialization.h>
#include <ze/common/macros.h>
#include <ze/common/types.h>

namespace ze {

//! Imu Model
class ImuModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuModel);

  typedef Eigen::Matrix<FloatType, 6, 1> measurement_t;

  ImuModel() = delete;

  ImuModel(const AccelerometerModel::Ptr accelerometer,
           const GyroscopeModel::Ptr gyroscope);

  //! Load an imu form a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& path);

  void setLabel(const std::string& label) { label_ = label; }
  void setId(const std::string& id) { id_ = id; }
  std::string label() const { return label_; }
  std::string id() const { return id_; }

  //! distort in place
  void distort(measurement_t* in) const;

  //! undistort in place
  void undistort(measurement_t* in) const;

  // getters
  inline const AccelerometerModel::Ptr accelerometerModel() const
  {
    return accelerometerModel_;
  }
  inline const GyroscopeModel::Ptr gyroscopeModel() const
  {
    return gyroscopeModel_;
  }

private:
  std::string id_;
  std::string label_;

  const AccelerometerModel::Ptr accelerometerModel_;
  const GyroscopeModel::Ptr gyroscopeModel_;
};

} // namespace ze
