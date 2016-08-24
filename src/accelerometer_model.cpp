// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include "ze/imu/accelerometer_model.h"

namespace ze {

AccelerometerModel::AccelerometerModel(
    const ImuIntrinsicModel::Ptr intrinsicModel, const ImuNoiseModel::Ptr noiseModel)
  : intrinsicModel_(intrinsicModel)
  , noiseModel_(noiseModel)
{
}

Vector3 AccelerometerModel::distort(const Eigen::Ref<const measurement_t>& a,
                                    const Eigen::Ref<const measurement_t>& w)
const
{
  return intrinsicModel_->distort(a, w);
}

Vector3 AccelerometerModel::undistort(const Eigen::Ref<const measurement_t>& a,
                                      const Eigen::Ref<const measurement_t>& w)
const
{
  return intrinsicModel_->undistort(a, w);
}

} // namespace ze
