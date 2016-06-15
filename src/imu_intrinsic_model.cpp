#include "ze/imu/imu_intrinsic_model.h"

namespace ze {

//------------------------------------------------------------------------------
// Intrinsics Base Class
ImuIntrinsicModel::ImuIntrinsicModel(ImuIntrinsicType type)
  : ImuIntrinsicModel(type, 0.0, UndefinedRange)
{
}

ImuIntrinsicModel::ImuIntrinsicModel(ImuIntrinsicType type, FloatType delay, FloatType range)
 : type_(type), delay_(delay), range_(range)
{
  CHECK(range == UndefinedRange || range > 0) << "Range must either be of constant UndefinedRange or be > 0";
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
  : ImuIntrinsicModel(Type, 0.0, ImuIntrinsicModel::UndefinedRange)
{
}

ImuIntrinsicModelCalibrated::ImuIntrinsicModelCalibrated(FloatType delay, FloatType range)
  : ImuIntrinsicModel(Type, delay, range)
{
}

void ImuIntrinsicModelCalibrated::undistort(Eigen::Ref<measurement_t> in) const
{
  //! @todo
  //! The calibrated model assumes that all relevant deterministic effects have been taken care of by the manufacturer.
  //! Hence, the mapping would be an identity.
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
  : ImuIntrinsicModel(Type, delay, range)
  , b_(b)
  , M_(M)
  , M_inverse_(M.inverse())
{
	CHECK(std::fabs(M_.determinant()) > 1.e-10)
	  << "M must be invertible. Its determinant evaluates to " << M_.determinant();
}

void ImuIntrinsicModelScaleMisalignment::undistort(Eigen::Ref<measurement_t> in) const
{
  in = M_inverse_ * (in - b_);
}

void ImuIntrinsicModelScaleMisalignment::distort(Eigen::Ref<measurement_t> in) const
{
  in = M_ * in + b_;
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment g-Sensitivity
ImuIntrinsicModelScaleMisalignmentGSensitivity::ImuIntrinsicModelScaleMisalignmentGSensitivity(
    FloatType delay,
    FloatType range,
    const Vector3& b,
    const Matrix3& M,
    const Matrix3& Ma)
  : ImuIntrinsicModel(Type, delay, range)
  , b_(b)
  , M_(M)
  , Ma_(Ma)
{
}

void ImuIntrinsicModelScaleMisalignmentGSensitivity::undistort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
  //! This model would need linear accelerations in addition to angular velocities.
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
  : ImuIntrinsicModel(Type, delay, range)
  , b_(b)
  , M_(M)
  , R_(R)
{
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::undistort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
  //! This model would need angular accelerations in addition.
}

void ImuIntrinsicModelScaleMisalignmentSizeEffect::distort(
    Eigen::Ref<measurement_t> in) const
{
  //! @todo
}

} // namespace ze
