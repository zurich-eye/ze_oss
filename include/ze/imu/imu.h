#pragma once

#include <cstdint>
#include <iostream>
#include <memory>
#include <type_traits>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>

#include <ze/common/types.h>
#include <ze/common/macros.h>
#include <ze/imu/imu_distortion.h>
#include <ze/imu/imu_distortion_scale_misalignment.h>

namespace ze {

/// \brief  6xD matrix of D imu measurements:
///         [ gyro_x[0],  gyro_x[1], ...  gyro_x[D-1]
///           gyro_y[0],  gyro_y[1], ...  gyro_y[D-1]
///           ...
///           accel_z[0], accel_z[1], ... accel_z[D-1] ].
//typedef Eigen::Matrix<double, 6, Eigen::Dynamic> GyroAccelData;

class Imu
{
public:
  ZE_POINTER_TYPEDEFS(Imu);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  virtual ~Imu() = default;

  static Ptr loadFromYaml(const std::string& path);

  void undistortMeasurements(ImuAccGyr& measurements);

  void printImu(void)
  {
    std::cout << label_ << std::endl;
  }

protected:

private:
  std::string label_;
};

}  // namespace ze
