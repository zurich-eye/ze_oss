#pragma once

#include <Eigen/LU>

#include <ze/common/logging.hpp>
#include <ze/common/macros.h>
#include <ze/common/matrix.h>
#include <ze/common/types.h>

namespace ze {

enum class ImuIntrinsicType
{
  Calibrated,
  ScaleMisalignment,
  ScaleMisalignmentGSensitivity,
  ScaleMisalignmentSizeEffect
};


//! Base Class for Intrinsic Models for both Accels and Gyros
class ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModel);

  typedef Eigen::Matrix<FloatType, -1, 1> primary_measurement_t;
  typedef Eigen::Matrix<FloatType, -1, 1> secondary_measurement_t;

  explicit ImuIntrinsicModel(ImuIntrinsicType type);
  ImuIntrinsicModel(ImuIntrinsicType type, FloatType delay, FloatType range);

  static constexpr FloatType UndefinedRange = -1.;

  inline ImuIntrinsicType type() const { return type_; }
  std::string typeAsString() const;

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
                          const Eigen::Ref<const secondary_measurement_t>& secondary) const = 0;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
                            const Eigen::Ref<const secondary_measurement_t>& secondary) const = 0;

  // getters
  inline FloatType delay() const { return delay_; }
  inline FloatType range() const { return range_; }

private:
  ImuIntrinsicType type_;
  FloatType delay_;
  FloatType range_;
};

//------------------------------------------------------------------------------
// Calibrated
class ImuIntrinsicModelCalibrated: public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelCalibrated);
  static constexpr ImuIntrinsicType Type = ImuIntrinsicType::Calibrated;

  using ImuIntrinsicModel::primary_measurement_t;
  using ImuIntrinsicModel::secondary_measurement_t;

  ImuIntrinsicModelCalibrated();
  ImuIntrinsicModelCalibrated(FloatType delay, FloatType range);

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
			  const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
			    const Eigen::Ref<const secondary_measurement_t>& secondary) const;
};

//------------------------------------------------------------------------------
// Scale Misalignment
class ImuIntrinsicModelScaleMisalignment : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignment);
  static constexpr ImuIntrinsicType Type = ImuIntrinsicType::ScaleMisalignment;

  using ImuIntrinsicModel::primary_measurement_t;
  using ImuIntrinsicModel::secondary_measurement_t;

  //! delay, range, bias, scale misalignment matrix
  ImuIntrinsicModelScaleMisalignment(FloatType delay, FloatType range,
                                     const Vector3& b, const Matrix3& M);

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
			  const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
			    const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  // getters
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }

private:
  Vector3 b_;
  Matrix3 M_;
  Matrix3 M_inverse_;
};

//------------------------------------------------------------------------------
// Scale Misalignment g-Sensitivity
class ImuIntrinsicModelScaleMisalignmentGSensitivity : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignmentGSensitivity);

  static constexpr ImuIntrinsicType Type =
      ImuIntrinsicType::ScaleMisalignmentGSensitivity;

  using ImuIntrinsicModel::primary_measurement_t;
  using ImuIntrinsicModel::secondary_measurement_t;

  //! delay, range, bias, scale misalignment matrix, g-sensitivity matrix
  ImuIntrinsicModelScaleMisalignmentGSensitivity(FloatType delay,
                                                 FloatType range,
                                                 const Vector3& b,
                                                 const Matrix3& M,
                                                 const Matrix3& Ma);

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
			  const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
			    const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  // getters
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }
  inline const Matrix3& Ma() const { return Ma_; }

private:
  Vector3 b_;
  Matrix3 M_;
  Matrix3 M_inverse_;
  Matrix3 Ma_;
};

//------------------------------------------------------------------------------
// Scale MisalignmentSize Effect
class ImuIntrinsicModelScaleMisalignmentSizeEffect : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignmentSizeEffect);
  static constexpr ImuIntrinsicType Type =
      ImuIntrinsicType::ScaleMisalignmentSizeEffect;

  using ImuIntrinsicModel::primary_measurement_t;
  using ImuIntrinsicModel::secondary_measurement_t;

  //! delay, range, bias, scale misalignment matrix, accel. column position vectors
  ImuIntrinsicModelScaleMisalignmentSizeEffect(FloatType delay,
                                               FloatType range,
                                               const Vector3& b,
                                               const Matrix3& M,
                                               const Matrix3& R);

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
			  const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
			    const Eigen::Ref<const secondary_measurement_t>& secondary) const;

  // getters
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }
  inline const Matrix3& R() const { return R_; }

private:
  Vector3 b_;
  Matrix3 M_;
  Matrix3 M_inverse_;
  Matrix3 R_;
};

} // namespace ze
