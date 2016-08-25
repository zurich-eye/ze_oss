// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <Eigen/LU>

#include <ze/common/logging.hpp>
#include <ze/common/macros.hpp>
#include <ze/common/matrix.hpp>
#include <ze/common/types.hpp>

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

  //! The inertial measurement models potentially depend on angular as well as
  //! linear quantities. Depending on whether a gyroscope or an accelerometer
  //! is modelled, the roles might be reversed. Accordingly, the 'primary'
  //! refers to the domain of the measurement that is being modelled (angular
  //! or linear) and the secondary on a measurement of the other domain
  //! respectively.

  typedef VectorX primary_measurement_t;
  typedef VectorX secondary_measurement_t;

  explicit ImuIntrinsicModel(ImuIntrinsicType type);
  ImuIntrinsicModel(ImuIntrinsicType type, real_t delay, real_t range);

  static constexpr real_t UndefinedRange = -1.;

  inline ImuIntrinsicType type() const { return type_; }
  std::string typeAsString() const;

  virtual Vector3 distort(const Eigen::Ref<const primary_measurement_t>& primary,
                          const Eigen::Ref<const secondary_measurement_t>& secondary) const = 0;

  virtual Vector3 undistort(const Eigen::Ref<const primary_measurement_t>& primary,
                            const Eigen::Ref<const secondary_measurement_t>& secondary) const = 0;

  // getters
  inline real_t delay() const { return delay_; }
  inline real_t range() const { return range_; }

private:
  ImuIntrinsicType type_;
  real_t delay_;
  real_t range_;
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
  ImuIntrinsicModelCalibrated(real_t delay, real_t range);

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
  ImuIntrinsicModelScaleMisalignment(real_t delay, real_t range,
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

  //! This model applies exclusively to gyroscopes.
  //! delay, range, bias, scale misalignment matrix, g-sensitivity matrix
  ImuIntrinsicModelScaleMisalignmentGSensitivity(real_t delay,
                                                 real_t range,
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

  //! This model applies exclusively to accelerometers.
  //! delay, range, bias, scale misalignment matrix, accel. column position vectors
  ImuIntrinsicModelScaleMisalignmentSizeEffect(real_t delay,
                                               real_t range,
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
