#include "ze/imu/imu_intrinsic_model.h"

namespace ze {

//----------------------------
// Intrinsics Base Class
ImuIntrinsicModel::ImuIntrinsicModel(IntrinsicModels type)
  : type_(type)
{
}

std::string ImuIntrinsicModel::typeAsString() const
{
  switch (type())
  {
    case Calibrated: return "Calibrated";
    case ScaleMisalignment: return "Scale Misalignment";
    case ScaleMisalignmentGSensitivity: return "Scale Misalignment g-Sensitivity";
    case ScaleMisalignmentSizeEffect: return "Scale Misalignment Size Effect";
    default:
      LOG(FATAL) << "Unknown intrinsics model";
  }

  return "";
}

//----------------------------
// Calibrated
ImuIntrinsicModelCalibrated::ImuIntrinsicModelCalibrated()
  : ImuIntrinsicModel(Type)
{
}

void ImuIntrinsicModelCalibrated::undistort(measurement_t* in) const
{
  //! @todo
}

void ImuIntrinsicModelCalibrated::distort(measurement_t* in) const
{
  //! @todo
}

//----------------------------
// Intrinsic Model Scale Misalignment
ImuIntrinsicModelScaleMisalignment::ImuIntrinsicModelScaleMisalignment(
    FloatType delay,
    uint32_t range,
    const Vector3& b,
    const Matrix3& M)
  : ImuIntrinsicModel(Type)
  , delay_(delay)
  , range_(range)
  , b_(b)
  , M_(M)
{
}

void ImuIntrinsicModelScaleMisalignment::undistort(measurement_t* in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignment::distort(measurement_t* in) const
{
  //! @todo
}

//----------------------------
// Intrinsic Model Scale Misalignment g-Sensitivity
ImuIntrinsicModelScaleMisalignmentGSensitivity::ImuIntrinsicModelScaleMisalignmentGSensitivity(
    FloatType delay,
    uint32_t range,
    const Vector3& b,
    const Matrix3& M,
    const Matrix3& Ma)
  : ImuIntrinsicModel(Type)
  , delay_(delay)
  , range_(range)
  , b_(b)
  , M_(M)
  , Ma_(Ma)
{
}

void ImuIntrinsicModelScaleMisalignmentGSensitivity::undistort(
    measurement_t* in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignmentGSensitivity::distort(
    measurement_t* in) const
{
  //! @todo
}

//----------------------------
// Intrinsic Model Scale Misalignment Size Effect
ImuIntrinsicModelScaleMisalignmentSizeEffect::ImuIntrinsicModelScaleMisalignmentSizeEffect(
    FloatType delay,
    uint32_t range,
    const Vector3& b,
    const Matrix3& M,
    const Matrix3& R)
  : ImuIntrinsicModel(Type)
  , delay_(delay)
  , range_(range)
  , b_(b)
  , M_(M)
  , R_(R)
{
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::undistort(
    measurement_t* in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::distort(
    measurement_t* in) const
{
  //! @todo
}

} // namespace ze
