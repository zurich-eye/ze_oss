#pragma once

#include <ze/imu/imu_intrinsic_model.h>
#include <ze/imu/imu_noise_model.h>
#include <ze/common/macros.h>
#include <ze/common/types.h>
namespace ze {

//! Gyroscope Model
class GyroscopeModel
{
public:
  ZE_POINTER_TYPEDEFS(GyroscopeModel);

  typedef Eigen::Matrix<FloatType, -1, 1> measurement_t;

  GyroscopeModel() = delete;

  GyroscopeModel(ImuIntrinsicModel::Ptr intrinsicModel,
                 ImuNoiseModel::Ptr noiseModel);

  Vector3 distort(const Eigen::Ref<const measurement_t>& w,
               const Eigen::Ref<const measurement_t>& a) const;
  Vector3 undistort(const Eigen::Ref<const measurement_t>& w,
                 const Eigen::Ref<const measurement_t>& a) const;

  // getters
  inline const ImuNoiseModel::Ptr noiseModel() const { return noiseModel_; }
  inline const ImuIntrinsicModel::Ptr intrinsicModel() const { return intrinsicModel_; }

private:
  const ImuIntrinsicModel::Ptr intrinsicModel_;
  const ImuNoiseModel::Ptr noiseModel_;
};

} // namespace ze
