#include "ze/imu/gyroscope_model.h"

namespace ze {

//----------------------------
// White brownian noise model
GyroscopeModel::GyroscopeModel(
    const ImuIntrinsicModel::Ptr intrinsicModel,
    const ImuNoiseModel::Ptr noiseModel)
  : intrinsicModel_(intrinsicModel)
  , noiseModel_(noiseModel)
{
}

void GyroscopeModel::distort(Eigen::Ref<measurement_t> in) const
{
  intrinsicModel_->distort(in);
}

void GyroscopeModel::undistort(Eigen::Ref<measurement_t> in) const
{
  intrinsicModel_->undistort(in);
}

} // namespace ze
