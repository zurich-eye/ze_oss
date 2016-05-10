// INSPIRED BY GTSAM

#pragma once

#include <functional>
#include <Eigen/Dense>
#include <ze/common/types.h>

namespace ze {

// For now, the manifold traits are primarily used for simplified computation of
// numerical derivatives (see numerical_derivative.h). We use the numerical
// derivatives mainly to test the correctness of the analytical derivatives.
template<typename T> struct traits;

// -----------------------------------------------------------------------------
// Manifold traits for scalars.
namespace internal {
template<typename Scalar>
struct ScalarTraits
{
  enum { dimension = 1 }; //! @todo(cfo): static constexpr int fails on GCC 4.8, works for Eigen because no derived class.
  typedef Eigen::Matrix<Scalar, 1, 1> TangentVector;
  typedef Eigen::Matrix<Scalar, 1, 1> Jacobian;

  static bool equals(Scalar v1, Scalar v2, Scalar tol = 1e-8)
  {
    return std::abs(v1 - v2) < tol;
  }

  static TangentVector local(const Scalar origin, Scalar other,
                             Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      (*H1)[0] = -1.0; // dlocal(origin, other) / dorigin
    }
    if (H2)
    {
      (*H2)[0] =  1.0; // dlocal(origin, other) / dother
    }
    TangentVector result;
    result(0) = other - origin;
    return result;
  }

  static Scalar retract(const Scalar origin, const TangentVector& v,
                        Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      (*H1)[0] = 1.0; // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      (*H2)[0] = 1.0; // dretract(origin, v) / dv
    }
    return origin + v[0];
  }
};
} // namespace internal

// Define scalar traits for float and double
template<> struct traits<double> : public internal::ScalarTraits<double> {};
template<> struct traits<float> : public internal::ScalarTraits<float> {};


// -----------------------------------------------------------------------------
// Manifold traits for fixed-size Eigen matrices and vectors with double precision.
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<FloatType, M, N, Options, MaxRows, MaxCols> >
{
  //static constexpr int dimension = M * N;
  enum { dimension = M * N };
  typedef Eigen::Matrix<FloatType, M, N, Options, MaxRows, MaxCols> Matrix;
  typedef Eigen::Matrix<FloatType, dimension, 1> TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;

  static bool equals(const Matrix& v1, const Matrix& v2, FloatType tol = 1e-8)
  {
    if (v1.size() != v2.size())
    {
      return false;
    }
    return (v1 - v2).array().abs().maxCoeff() < tol;
    // TODO(cfo): Check for nan entries.
  }

  static TangentVector local(
      const Matrix& origin, Matrix other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = -Jacobian::Identity(); // dLocal(origin, other)/dOrigin
    }
    if (H2)
    {
      *H2 =  Jacobian::Identity(); // dLocal(origin, other)/dOther
    }
    TangentVector result;
    Eigen::Map<Matrix>(result.data()) = other - origin;
    return result;
  }

  static Matrix retract(
      const Matrix& origin, const TangentVector& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = Jacobian::Identity(); // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      *H2 = Jacobian::Identity(); // dretract(origin, v) / dv
    }
    return origin + Eigen::Map<const Matrix>(v.data());
  }
};

// -----------------------------------------------------------------------------
// Manifold traits for dynamic-size Eigen matrices and vectors with double precision.
namespace internal {

// traits for dynamic Eigen matrices
template<int M, int N, int Options, int MaxRows, int MaxCols>
struct DynamicMatrixTraits {

  typedef Eigen::Matrix<FloatType, M, N, Options, MaxRows, MaxCols> DynamicMatrix;

  enum Dimension : int { dimension = Eigen::Dynamic };

  typedef VectorX TangentVector;
  typedef Eigen::Matrix<FloatType, dimension, dimension> Jacobian;
  typedef DynamicMatrix ManifoldType;

  static int getDimension(const DynamicMatrix& m)
  {
    return m.rows() * m.cols();
  }

  static Jacobian eye(const DynamicMatrix& m)
  {
    int dim = getDimension(m);
    return Eigen::Matrix<FloatType, dimension, dimension>::Identity(dim, dim);
  }

  static TangentVector local(
      const DynamicMatrix& origin, const DynamicMatrix& other,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = -eye(origin); // dlocal(origin, other) / dorigin
    }
    if (H2)
    {
      *H2 =  eye(origin); // dlocal(origin, other) / dother
    }
    TangentVector v(getDimension(origin));
    Eigen::Map<DynamicMatrix>(v.data(), origin.rows(), origin.cols()) = other - origin;
    return v;
  }

  static DynamicMatrix retract(
      const DynamicMatrix& origin, const TangentVector& v,
      Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if (H1)
    {
      *H1 = eye(origin); // dretract(origin, v) / dorigin
    }
    if (H2)
    {
      *H2 = eye(origin); // dretract(origin, v) / dv
    }
    return origin + Eigen::Map<const DynamicMatrix>(v.data(), origin.rows(), origin.cols());
  }
};

} // namespace internal

// traits for fully dynamic matrix
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<Eigen::Dynamic, Eigen::Dynamic, Options, MaxRows, MaxCols> {
};

// traits for dynamic column vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<FloatType, Eigen::Dynamic, 1, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<Eigen::Dynamic, 1, Options, MaxRows, MaxCols> {
};

// traits for dynamic row vector
template<int Options, int MaxRows, int MaxCols>
struct traits<Eigen::Matrix<FloatType, 1, Eigen::Dynamic, Options, MaxRows, MaxCols> > :
    public internal::DynamicMatrixTraits<1, Eigen::Dynamic, Options, MaxRows, MaxCols> {
};

} // namespace ze



