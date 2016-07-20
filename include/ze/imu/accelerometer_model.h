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

  typedef Eigen::Matrix<FloatType, -1, 1> measurement_t;

  AccelerometerModel() = delete;

  AccelerometerModel(ImuIntrinsicModel::Ptr intrinsicModel,
                     ImuNoiseModel::Ptr noiseModel);

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
