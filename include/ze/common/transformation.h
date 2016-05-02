#pragma once

#include <limits>
#include <vector>
#include <kindr/minimal/quat-transformation.h>
#include <ze/common/types.h>
#include <ze/common/matrix.h>

namespace ze {

using Transformation = kindr::minimal::QuatTransformationTemplate<FloatType>;
using Quaternion = kindr::minimal::RotationQuaternionTemplate<FloatType>;
using AngleAxis = kindr::minimal::AngleAxisTemplate<FloatType>;

using TransformationVector = std::vector<Transformation, Eigen::aligned_allocator<Transformation>>;
using QuaternionVector = std::vector<Transformation, Eigen::aligned_allocator<Quaternion>>;

// -----------------------------------------------------------------------------
// Transformation utils

// Right Jacobian for Exponential map in SO(3)
inline Matrix3 expmapDerivativeSO3(const Vector3& omega)
{
  FloatType theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<FloatType>::epsilon())
  {
    return I_3x3;
  }
  FloatType theta = std::sqrt(theta2);  // rotation angle
  // element of Lie algebra so(3): X = omega^, normalized by normx
  const Matrix3 Y = skewSymmetric(omega) / theta;
  return I_3x3 - ((1 - std::cos(theta)) / (theta)) * Y
               + (1 - std::sin(theta) / theta) * Y * Y;
}

// Right Jacobian for Log map in SO(3)
inline Matrix3 logmapDerivativeSO3(const Vector3& omega)
{
  FloatType theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<FloatType>::epsilon())
  {
    return I_3x3;
  }
  FloatType theta = std::sqrt(theta2);  // rotation angle
  const Matrix3 X = skewSymmetric(omega); // element of Lie algebra so(3): X = omega^
  return I_3x3 + 0.5 * X
      + (1 / (theta * theta) - (1 + std::cos(theta)) / (2 * theta * std::sin(theta))) * X * X;
}

// -----------------------------------------------------------------------------
// Type traits used for optimization
template<typename T> struct traits;

// -----------------------------------------------------------------------------
// Manifold traits for SO(3)
template<> struct traits<Quaternion>
{
  enum { dimension = 3 }; // The dimension of the manifold.

  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  static bool equals(
      const Quaternion& q1, const Quaternion& q2, FloatType tol = 1e-8)
  {
    return (q1.getUnique().vector()
            - q2.getUnique().vector()).array().abs().maxCoeff() < tol;
  }

  static TangentVector local(
      const Quaternion& origin, const Quaternion& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion h = origin.inverse() * other;
    const TangentVector v = h.log();
    if(H1 || H2)
    {
      Jacobian D_v_h = logmapDerivativeSO3(v);
      if(H1)
      {
        // dlocal(origin, other) / dorigin, using that Adjoint(h.inverse()) = h.inverse()
        *H1 = - D_v_h * h.inverse().getRotationMatrix();
      }
      if(H2)
      {
        // dlocal(origin, other) / dother
        *H2 = D_v_h;
      }
    }
    return v;
  }

  static Quaternion retract(
      const Quaternion& origin, const Vector3& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion g = Quaternion::exp(v);
    const Quaternion h = origin * g;
    if (H1)
    {
      // dretract(origin, v) / dorigin
      *H1 = g.inverse().getRotationMatrix(); // Adjoint(g.inverse()) = g.inverse()
    }
    if (H2)
    {
      // dretract(origin, v) / dv
      *H2 = expmapDerivativeSO3(v);
    }
    return h;
  }
};

// -----------------------------------------------------------------------------
// Manifold traits for SE(3)
template<> struct traits<Transformation>
{
  enum { dimension = 6 }; // The dimension of the manifold.

  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  static bool equals(
      const Transformation& T1, const Transformation& T2, FloatType tol = 1e-8)
  {
    return (T1.getRotation().getUnique().vector()
            - T2.getRotation().getUnique().vector()).array().abs().maxCoeff() < tol
        && (T1.getPosition() - T2.getPosition()).array().abs().maxCoeff() < tol;
  }

  static TangentVector local(
      const Transformation& origin, const Transformation& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Transformation h = origin.inverse() * other;
    const TangentVector v = (Vector6() << h.getPosition(), h.getRotation().log()).finished();
    if(H1 || H2)
    {
      LOG(FATAL) << "Not implemented";
    }
    return v;
  }

  static Transformation retract(
      const Transformation& origin, const Vector6& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    Transformation g(Quaternion::exp(v.tail<3>()), v.head<3>()); // Chart at origin
    Transformation h  = origin * g;
    if(H1 || H2)
    {
      LOG(FATAL) << "Not implemented";
    }
    return h;
  }
};

} // namespace ze
