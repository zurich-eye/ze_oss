// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/imu/imu_intrinsic_model.h>
#include <ze/imu/imu_noise_model.h>
#include <ze/common/macros.h>
#include <ze/common/types.h>

namespace ze {

//! Accelerometer Model
class AccelerometerModel
{
public:
  ZE_POINTER_TYPEDEFS(AccelerometerModel);

  typedef VectorX measurement_t;

  AccelerometerModel() = delete;

  AccelerometerModel(ImuIntrinsicModel::Ptr intrinsicModel,
                     ImuNoiseModel::Ptr noiseModel);

  //! These models may depend on both angular and linear quantities as well as
  //! higher order time derivatives of the quantities. A measurement is
  //! composed exclusively of either angular or linear quantities and features
  //! time derivatives in increasing order starting from 0.
  Vector3 distort(const Eigen::Ref<const measurement_t>& a,
                  const Eigen::Ref<const measurement_t>& w) const;
  Vector3 undistort(const Eigen::Ref<const measurement_t>& a,
                    const Eigen::Ref<const measurement_t>& w) const;

  // getters
  inline const ImuNoiseModel::Ptr noiseModel() const { return noiseModel_; }
  inline const ImuIntrinsicModel::Ptr intrinsicModel() const
  {
    return intrinsicModel_;
  }

private:
  const ImuIntrinsicModel::Ptr intrinsicModel_;
  const ImuNoiseModel::Ptr noiseModel_;
};

} // namespace ze
