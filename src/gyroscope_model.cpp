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

Vector3 GyroscopeModel::distort(const Eigen::Ref<const measurement_t>& w,
                                const Eigen::Ref<const measurement_t>& a) const
{
  return intrinsicModel_->distort(w, a);
}

Vector3 GyroscopeModel::undistort(const Eigen::Ref<const measurement_t>& w,
                                  const Eigen::Ref<const measurement_t>& a) const
{
  return intrinsicModel_->undistort(w, a);
}

} // namespace ze
