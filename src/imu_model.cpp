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

void ImuModel::distort(Eigen::Ref<measurement_t> in) const
{
  accelerometerModel_->distort(in.head<3>());
  gyroscopeModel_->distort(in.tail<3>());
}

void ImuModel::undistort(Eigen::Ref<measurement_t> in) const
{
  accelerometerModel_->undistort(in.head<3>());
  gyroscopeModel_->undistort(in.tail<3>());
}

} // namespace ze
