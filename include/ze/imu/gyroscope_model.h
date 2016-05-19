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

  typedef Eigen::Matrix<FloatType, 3, 1> measurement_t;

  GyroscopeModel() = delete;

  GyroscopeModel(ImuIntrinsicModel::Ptr intrinsicModel,
                 ImuNoiseModel::Ptr noiseModel);

  //! distort in place
  void distort(Eigen::Ref<measurement_t> in) const;

  //! undistort in place
  void undistort(Eigen::Ref<measurement_t> in) const;

  // getters
  inline const ImuNoiseModel::Ptr noiseModel() const { return noiseModel_; }
  inline const ImuIntrinsicModel::Ptr intrinsicModel() const { return intrinsicModel_; }

private:
  const ImuIntrinsicModel::Ptr intrinsicModel_;
  const ImuNoiseModel::Ptr noiseModel_;
};

} // namespace ze
