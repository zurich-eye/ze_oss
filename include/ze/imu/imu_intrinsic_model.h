#pragma once

#include <ze/common/logging.hpp>
#include <ze/common/macros.h>
#include <ze/common/types.h>

namespace ze {

//! Base Class for Intrinsic Models for both Accels and Gyros
class ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModel);

  enum IntrinsicModels
  {
    Calibrated,
    ScaleMisalignment,
    ScaleMisalignmentGSensitivity,
    ScaleMisalignmentSizeEffect
  };

  explicit ImuIntrinsicModel(IntrinsicModels type);

  typedef Eigen::Matrix<FloatType, 3, 1> measurement_t;

  inline IntrinsicModels type() const { return type_; }
  std::string typeAsString() const;

  //! distort in place
  virtual void distort(measurement_t* in) const = 0;

  //! undistort in place
  virtual void undistort(measurement_t* in) const = 0;

private:
  IntrinsicModels type_;
};

//---------------------------------------------
// Calibrated
class ImuIntrinsicModelCalibrated: public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelCalibrated);
  static constexpr IntrinsicModels Type = ImuIntrinsicModel::Calibrated;

  ImuIntrinsicModelCalibrated();

  //! distort in place
  virtual void distort(measurement_t* in) const;

  //! undistort in place
  virtual void undistort(measurement_t* in) const;
};

//---------------------------------------------
// Scale Misalignment
class ImuIntrinsicModelScaleMisalignment : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignment);
  static constexpr IntrinsicModels Type = ImuIntrinsicModel::ScaleMisalignment;

  //! delay, range, bias, scale misalignment matrix
  ImuIntrinsicModelScaleMisalignment(FloatType delay, FloatType range,
                                     const Vector3& b, const Matrix3& M);

  //! distort in place
  virtual void distort(measurement_t* in) const;

  //! undistort in place
  virtual void undistort(measurement_t* in) const;

  // getters
  inline FloatType delay() const { return delay_; }
  inline uint32_t range() const { return range_; }
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }

private:
  FloatType delay_;
  FloatType range_;
  Vector3 b_;
  Matrix3 M_;
};

//---------------------------------------------
// Scale Misalignment g-Sensitivity
class ImuIntrinsicModelScaleMisalignmentGSensitivity : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignmentGSensitivity);

  static constexpr IntrinsicModels Type =
      ImuIntrinsicModel::ScaleMisalignmentGSensitivity;

  //! delay, range, bias, scale misalignment matrix, g-sensitivity matrix
  ImuIntrinsicModelScaleMisalignmentGSensitivity(FloatType delay,
                                                 FloatType range,
                                                 const Vector3& b,
                                                 const Matrix3& M,
                                                 const Matrix3& Ma);
  //! distort in place
  virtual void distort(measurement_t* in) const;

  //! undistort in place
  virtual void undistort(measurement_t* in) const;

  // getters
  inline FloatType delay() const { return delay_; }
  inline uint32_t range() const { return range_; }
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }
  inline const Matrix3& Ma() const { return Ma_; }

private:
  FloatType delay_;
  FloatType range_;
  Vector3 b_;
  Matrix3 M_;
  Matrix3 Ma_;
};

//---------------------------------------------
// Scale MisalignmentSize Effect
class ImuIntrinsicModelScaleMisalignmentSizeEffect : public ImuIntrinsicModel
{
public:
  ZE_POINTER_TYPEDEFS(ImuIntrinsicModelScaleMisalignmentSizeEffect);
  static constexpr IntrinsicModels Type =
      ImuIntrinsicModel::ScaleMisalignmentSizeEffect;

  //! delay, range, bias, scale misalignment matrix, accel. column position vectors
  ImuIntrinsicModelScaleMisalignmentSizeEffect(FloatType delay,
                                               FloatType range,
                                               const Vector3& b,
                                               const Matrix3& M,
                                               const Matrix3& R);
  //! distort in place
  virtual void distort(measurement_t* in) const;

  //! undistort in place
  virtual void undistort(measurement_t* in) const;

  // getters
  inline FloatType delay() const { return delay_; }
  inline FloatType range() const { return range_; }
  inline const Vector3& b() const { return b_; }
  inline const Matrix3& M() const { return M_; }
  inline const Matrix3& R() const { return R_; }

private:
  FloatType delay_;
  FloatType range_;
  Vector3 b_;
  Matrix3 M_;
  Matrix3 R_;
};

} // namespace ze
