#pragma once

#include <Eigen/Dense>
#include <gflags/gflags.h>

#include <ze/common/macros.h>
#include <ze/common/types.h>
#include <ze/imu/imu.h>

namespace ze {

class ImuDistortion
{
public:
  ZE_POINTER_TYPEDEFS(ImuDistortion);

  enum class Type {
    kIdeal = 0,
    kScaleMisalignment = 1,
    kScaleMisalignmentSizeEffect = 2
  };

  virtual ~ImuDistortion() = default;

  virtual void undistort(ImuAccGyr& measurements);

protected:
  ImuDistortion(Type distortion_type);

  inline Type getDistortionType()
  {
    return distortion_type_;
  }

protected:

  Type distortion_type_;
  VectorX distortion_params_;

};

}  // namespace ze
