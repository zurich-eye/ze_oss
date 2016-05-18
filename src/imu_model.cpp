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

void ImuModel::distort(measurement_t* in) const
{
  //! @todo
}

void ImuModel::undistort(ImuModel::measurement_t* in) const
{
  //! @todo
}

} // namespace ze
