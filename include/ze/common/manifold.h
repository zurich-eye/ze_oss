// INSPIRED BY GTSAM

#pragma once

#include <functional>
#include <Eigen/Dense>

namespace ze {

template<typename T> struct traits;

// -----------------------------------------------------------------------------
namespace internal {
template<typename Scalar>
struct ScalarTraits
{
  static constexpr int dimension = 1;
  typedef Eigen::Matrix<double, 1, 1> TangentVector;
  typedef Eigen::Matrix<double, 1, 1> Jacobian;

  static bool Equals(Scalar v1, Scalar v2, double tol = 1e-8)
  {
    return std::abs(v1 - v2) < tol;
  }

  static TangentVector Local(Scalar origin, Scalar other,
                             Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1) (*H1)[0] = -1.0;
    if (H2) (*H2)[0] =  1.0;
    TangentVector result;
    result(0) = other - origin;
    return result;
  }

  static Scalar Retract(Scalar origin, const TangentVector& v,
                        Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1) (*H1)[0] = 1.0;
    if (H2) (*H2)[0] = 1.0;
    return origin + v[0];
  }
};
} // namespace internal

// Define scalar traits for float and double
template<> struct traits<double> : public internal::ScalarTraits<double> {};
template<> struct traits<float> : public internal::ScalarTraits<float> {};


// -----------------------------------------------------------------------------
// traits for any fixed double Eigen matrix
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> >
{
  static constexpr int dimension = M * N;
  typedef Eigen::Matrix<double, M, N, Options, MaxRows, MaxCols> Matrix;
  typedef Eigen::Matrix<double, dimension, 1> TangentVector;
  typedef Eigen::Matrix<double, dimension, dimension> Jacobian;

  static bool Equals(const Matrix& v1, const Matrix& v2, double tol = 1e-8)
  {
    if (v1.size() != v2.size())
      return false;
    return (v1 - v2).array().abs().maxCoeff() < tol;
    // TODO(cfo): Check for nan entries.
  }

  static TangentVector Local(Matrix origin, Matrix other,
                             Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1) (*H1) = -Jacobian::Identity();
    if (H2) (*H2) =  Jacobian::Identity();
    TangentVector result;
    Eigen::Map<Matrix>(result.data()) = other - origin;
    return result;
  }

  static Matrix Retract(Matrix origin, const TangentVector& v,
                        Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1) (*H1) = Jacobian::Identity();
    if (H2) (*H2) = Jacobian::Identity();
    return origin + Eigen::Map<const Matrix>(v.data());
  }
};

} // namespace ze



