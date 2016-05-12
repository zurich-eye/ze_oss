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

  typedef Eigen::Matrix<FloatType, 3, 1> measurement_t;

  AccelerometerModel() = delete;

  AccelerometerModel(ImuIntrinsicModel::Ptr intrinsicModel,
                     ImuNoiseModel::Ptr noiseModel);

  //! distort in place
  void distort(measurement_t* in) const;

  //! undistort in place
  void undistort(measurement_t* in) const;

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
