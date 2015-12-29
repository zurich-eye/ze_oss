#pragma once

#include <limits>
#include <kindr/minimal/quat-transformation.h>

namespace ze {

using Transformation = kindr::minimal::QuatTransformation;
using Quaternion = kindr::minimal::RotationQuaternion;

Eigen::Matrix3d skewSymmetric(Eigen::Vector3d xyz)
{
  return (Eigen::Matrix3d() <<
          0.0, -xyz.z(), +xyz.y(),
          +xyz.z(), 0.0, -xyz.x(),
          -xyz.y(), +xyz.x(), 0.0).finished();
}

// Right Jacobian for Exponential map in SO(3) - equation (10.86) and following
// equations in G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie
// Groups", Volume 2, 2008.
// expmap(thetahat + omega) \approx expmap(thetahat) * expmap(Jr * omega)
// where Jr = ExpmapDerivative(thetahat);
Eigen::Matrix3d expmapDerivativeSO3(const Eigen::Vector3d& omega)
{
  double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon())
    return Eigen::Matrix3d::Identity();
  double theta = std::sqrt(theta2);  // rotation angle
  // element of Lie algebra so(3): X = omega^, normalized by normx
  const Eigen::Matrix3d Y = skewSymmetric(omega) / theta;
  return Eigen::Matrix3d::Identity()
      - ((1 - cos(theta)) / (theta)) * Y
      + (1 - sin(theta) / theta) * Y * Y;
}

// Right Jacobian for Log map in SO(3) - equation (10.86) and following equations
// in G.S. Chirikjian, "Stochastic Models, Information Theory, and Lie Groups",
// Volume 2, 2008.
// logmap( Rhat * expmap(omega) ) \approx logmap( Rhat ) + Jrinv * omega
// where Jrinv = LogmapDerivative(omega);
Eigen::Matrix3d logmapDerivativeSO3(const Eigen::Vector3d& omega)
{
  double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon())
    return Eigen::Matrix3d::Identity();
  double theta = std::sqrt(theta2);  // rotation angle
  const Eigen::Matrix3d X = skewSymmetric(omega); // element of Lie algebra so(3): X = omega^
  return Eigen::Matrix3d::Identity()
      + 0.5 * X
      + (1 / (theta * theta) - (1 + std::cos(theta)) / (2 * theta * std::sin(theta))) * X * X;
}


template<typename T> struct traits;

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

} // namespace ze
