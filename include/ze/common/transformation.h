#pragma once

#include <limits>
#include <vector>
#include <kindr/minimal/quat-transformation.h>
#include <ze/common/types.h>
#include <ze/common/matrix.h>

namespace ze {

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;
using AngleAxis = kindr::minimal::AngleAxis;

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
  static constexpr int dimension = 3; // The dimension of the manifold.

  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;

  static bool Equals(const Quaternion& q1, const Quaternion& q2, double tol = 1e-8)
  {
    return (q1.getUnique().vector() - q2.getUnique().vector()).array().abs().maxCoeff() < tol;
  }

  static TangentVector Local(const Quaternion& origin, const Quaternion& other,
                             Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion h = origin.inverse() * other;
    const TangentVector v = h.log();
    if(H1 || H2)
    {
      Jacobian D_v_h = logmapDerivativeSO3(v);
      if(H1) *H1 = - D_v_h * h.inverse().getRotationMatrix(); // Adjoint(h.inverse()) = h.inverse()
      if(H2) *H2 = D_v_h;
    }
    return v;
  }

  static Quaternion Retract(const Quaternion& origin, const Eigen::Vector3d& d,
                            Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    const Quaternion g = Quaternion::exp(d);
    const Quaternion h = origin * g;
    if (H1) *H1 = g.inverse().getRotationMatrix(); // Adjoint(g.inverse()) = g.inverse()
    if (H2) *H2 = expmapDerivativeSO3(d);
    return h;
  }
};

// -----------------------------------------------------------------------------
// Manifold traits for SE(3)

//
// TODO(cfo): SE(3) traits.
//

} // namespace ze
