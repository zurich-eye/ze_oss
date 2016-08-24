// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include "ze/imu/imu_model.h"

namespace ze {

ImuModel::ImuModel(
    const AccelerometerModel::Ptr accelerometerModel,
    const GyroscopeModel::Ptr gyroscopeModel)
  : accelerometerModel_(accelerometerModel)
  , gyroscopeModel_(gyroscopeModel)
{
}

ImuModel::Ptr ImuModel::loadFromYaml(const std::string& path)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(path.c_str());
    return doc.as<ImuModel::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Failed to load IMU from file " << path << " with the error: \n"
               << ex.what();
  }
  return ImuModel::Ptr();
}

Vector6 ImuModel::distort(const Eigen::Ref<const measurement_t>& primary,
                          const Eigen::Ref<const measurement_t>& secondary) const
{
  Vector6 out;
  out.head<3>() = accelerometerModel_->distort(primary, secondary);
  out.tail<3>() = gyroscopeModel_->distort(secondary, primary);
  return out;
}

Vector6 ImuModel::undistort(const Eigen::Ref<const measurement_t>& primary,
                            const Eigen::Ref<const measurement_t>& secondary) const
{
  Vector6 out;
  out.head<3>() = accelerometerModel_->undistort(primary, secondary);
  out.tail<3>() = gyroscopeModel_->undistort(secondary, primary);
  return out;
}

} // namespace ze
