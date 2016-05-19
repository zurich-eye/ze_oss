#include "ze/imu/imu_intrinsic_model.h"

namespace ze {

//------------------------------------------------------------------------------
// Intrinsics Base Class
ImuIntrinsicModel::ImuIntrinsicModel(ImuIntrinsicType type)
  : type_(type)
{
}

std::string ImuIntrinsicModel::typeAsString() const
{
  switch (type())
  {
    case ImuIntrinsicType::Calibrated: return "Calibrated";
    case ImuIntrinsicType::ScaleMisalignment: return "Scale Misalignment";
    case ImuIntrinsicType::ScaleMisalignmentGSensitivity: return "Scale Misalignment g-Sensitivity";
    case ImuIntrinsicType::ScaleMisalignmentSizeEffect: return "Scale Misalignment Size Effect";
    default:
      LOG(FATAL) << "Unknown intrinsics model";
  }

  return "";
}

//------------------------------------------------------------------------------
// Calibrated
ImuIntrinsicModelCalibrated::ImuIntrinsicModelCalibrated()
  : ImuIntrinsicModel(Type)
{
}

void ImuIntrinsicModelCalibrated::undistort(Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

void ImuIntrinsicModelCalibrated::distort(Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment
ImuIntrinsicModelScaleMisalignment::ImuIntrinsicModelScaleMisalignment(
    FloatType delay,
    FloatType range,
    const Vector3& b,
    const Matrix3& M)
  : ImuIntrinsicModel(Type)
  , delay_(delay)
  , range_(range)
  , b_(b)
  , M_(M)
{
  CHECK(range > 0) << "Range must be > 0";
}

void ImuIntrinsicModelScaleMisalignment::undistort(Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignment::distort(Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment g-Sensitivity
ImuIntrinsicModelScaleMisalignmentGSensitivity::ImuIntrinsicModelScaleMisalignmentGSensitivity(
    FloatType delay,
    FloatType range,
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
  CHECK(range > 0) << "Range must be > 0";
}

void ImuIntrinsicModelScaleMisalignmentGSensitivity::undistort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignmentGSensitivity::distort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment Size Effect
ImuIntrinsicModelScaleMisalignmentSizeEffect::ImuIntrinsicModelScaleMisalignmentSizeEffect(
    FloatType delay,
    FloatType range,
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
  CHECK(range > 0) << "Range must be > 0";
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::undistort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::distort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

} // namespace ze
