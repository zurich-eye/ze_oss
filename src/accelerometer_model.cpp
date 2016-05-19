#include "ze/imu/accelerometer_model.h"

namespace ze {

AccelerometerModel::AccelerometerModel(
    const ImuIntrinsicModel::Ptr intrinsicModel, const ImuNoiseModel::Ptr noiseModel)
  : intrinsicModel_(intrinsicModel)
  , noiseModel_(noiseModel)
{
}

void AccelerometerModel::distort(Eigen::Ref<measurement_t> in) const
{
  intrinsicModel_->distort(in);
}

void AccelerometerModel::undistort(Eigen::Ref<measurement_t> in) const
{
  intrinsicModel_->undistort(in);
}

} // namespace ze
