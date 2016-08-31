// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/imu/imu_intrinsic_model.hpp>

namespace ze {

// The constexpr needs a definition to make it passable by reference.
constexpr real_t ImuIntrinsicModel::UndefinedRange;

//------------------------------------------------------------------------------
// Intrinsics Base Class
ImuIntrinsicModel::ImuIntrinsicModel(ImuIntrinsicType type)
  : ImuIntrinsicModel(type, 0.0, UndefinedRange)
{
}

ImuIntrinsicModel::ImuIntrinsicModel(ImuIntrinsicType type, real_t delay, real_t range)
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

ImuIntrinsicModelCalibrated::ImuIntrinsicModelCalibrated(real_t delay, real_t range)
  : ImuIntrinsicModel(Type, delay, range)
{
}

Vector3 ImuIntrinsicModelCalibrated::undistort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  //! The calibrated model assumes that all relevant deterministic effects have
  //! been taken care of by the manufacturer. Hence, the mapping is an identity.
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  return primary.head<3>();
}

Vector3 ImuIntrinsicModelCalibrated::distort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  //! The calibrated model assumes that all relevant deterministic effects have
  //! been taken care of by the manufacturer. Hence, the mapping is an identity.
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  return primary.head<3>();
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment
ImuIntrinsicModelScaleMisalignment::ImuIntrinsicModelScaleMisalignment(
    real_t delay,
    real_t range,
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

Vector3 ImuIntrinsicModelScaleMisalignment::undistort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  return M_inverse_ * (primary.head<3>() - b_);
}

Vector3 ImuIntrinsicModelScaleMisalignment::distort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  return M_ * primary.head<3>() + b_;
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment g-Sensitivity
ImuIntrinsicModelScaleMisalignmentGSensitivity::ImuIntrinsicModelScaleMisalignmentGSensitivity(
    real_t delay,
    real_t range,
    const Vector3& b,
    const Matrix3& M,
    const Matrix3& Ma)
  : ImuIntrinsicModel(Type, delay, range)
  , b_(b)
  , M_(M)
  , M_inverse_(M.inverse())
  , Ma_(Ma)
{
  CHECK(std::fabs(M_.determinant()) > 1.e-10)
    << "M must be invertible. Its determinant evaluates to " << M_.determinant();
}

Vector3 ImuIntrinsicModelScaleMisalignmentGSensitivity::undistort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  CHECK_GE(secondary.rows(), 3) << "Secondary model input has incorrect size.";
  Vector3 a = secondary.head<3>();
  Vector3 w = primary.head<3>();
  return M_inverse_ * (w - Ma_ * a - b_);
}

Vector3 ImuIntrinsicModelScaleMisalignmentGSensitivity::distort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  CHECK_GE(secondary.rows(), 3) << "Secondary model input has incorrect size.";
  Vector3 a = secondary.head<3>();
  Vector3 w = primary.head<3>();
  return M_ * w + Ma_ * a + b_;
}

//------------------------------------------------------------------------------
// Intrinsic Model Scale Misalignment Size Effect
ImuIntrinsicModelScaleMisalignmentSizeEffect::ImuIntrinsicModelScaleMisalignmentSizeEffect(
    real_t delay,
    real_t range,
    const Vector3& b,
    const Matrix3& M,
    const Matrix3& R)
  : ImuIntrinsicModel(Type, delay, range)
  , b_(b)
  , M_(M)
  , M_inverse_(M.inverse())
  , R_(R)
{
  CHECK(std::fabs(M_.determinant()) > 1.e-10)
    << "M must be invertible. Its determinant evaluates to " << M_.determinant();
}

Vector3 ImuIntrinsicModelScaleMisalignmentSizeEffect::undistort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  CHECK_GE(secondary.rows(), 6) << "Secondary model input has incorrect size.";
  Vector3 a = primary.head<3>();
  Vector3 w = secondary.head<3>();
  Vector3 w_dot = secondary.segment<3>(3);
  return M_inverse_ * (a - b_) - (skewSymmetric(w_dot) * R_ + skewSymmetric(w) * skewSymmetric(w) * R_).diagonal();
}

Vector3 ImuIntrinsicModelScaleMisalignmentSizeEffect::distort(
    const Eigen::Ref<const primary_measurement_t>& primary,
    const Eigen::Ref<const secondary_measurement_t>& secondary) const
{
  CHECK_GE(primary.rows(), 3) << "Primary model input has incorrect size.";
  CHECK_GE(secondary.rows(), 6) << "Secondary model input has incorrect size.";
  Vector3 a = primary.head<3>();
  Vector3 w = secondary.head<3>();
  Vector3 w_dot = secondary.segment<3>(3);
  return M_ * (a + (skewSymmetric(w_dot) * R_ + skewSymmetric(w) * skewSymmetric(w) * R_).diagonal()) + b_;
}

} // namespace ze
