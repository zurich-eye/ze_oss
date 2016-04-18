#include <ze/geometry/triangulation.h>

#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.h>

namespace ze {

Position triangulateNonLinear(
    const Transformation& T_A_B,
    const Eigen::Ref<const Bearing>& f_A,
    const Eigen::Ref<const Bearing>& f_B)
{
  Bearing f_A_hat = T_A_B.getRotation().rotate(f_B);
  Vector2 b(
        T_A_B.getPosition().dot(f_A),
        T_A_B.getPosition().dot(f_A_hat));
  Matrix2 A;
  A(0,0) = f_A.dot(f_A);
  A(1,0) = f_A.dot(f_A_hat);
  A(0,1) = -A(1,0);
  A(1,1) = -f_A_hat.dot(f_A_hat);
  Vector2 lambda = A.inverse() * b;
  Position xm = lambda(0) * f_A;
  Position xn = T_A_B.getPosition() + lambda(1) * f_A_hat;
  Position p_A(0.5 * ( xm + xn ));
  return p_A;
}

void triangulateManyAndComputeAngularErrors(
    const Transformation& T_A_B,
    const Bearings& f_A_vec,
    const Bearings& f_B_vec,
    Positions& p_A,
    VectorX& reprojection_erors)
{
  CHECK_EQ(f_A_vec.cols(), f_B_vec.cols());
  CHECK_EQ(f_A_vec.cols(), p_A.cols());
  CHECK_EQ(f_A_vec.cols(), reprojection_erors.size());
  const Transformation T_B_A = T_A_B.inverse();
  for (int i = 0; i < f_A_vec.cols(); ++i)
  {
    p_A.col(i) = triangulateNonLinear(T_A_B, f_A_vec.col(i), f_B_vec.col(i));
    Bearing f_A_predicted = p_A.col(i).normalized();
    Bearing f_B_predicted = (T_B_A * p_A.col(i)).normalized();

    // Bearing-vector based outlier criterium (select threshold accordingly):
    // 1 - (f1' * f2) = 1 - cos(alpha) as used in OpenGV.
    FloatType reproj_error_1 = 1.0 - (f_A_vec.col(i).dot(f_A_predicted));
    FloatType reproj_error_2 = 1.0 - (f_B_vec.col(i).dot(f_B_predicted));
    reprojection_erors(i) = reproj_error_1 + reproj_error_2;
  }
}

std::pair<Vector4, bool> triangulateHomogeneousDLT(
    const TransformationVector& T_C_W,
    const Bearings& f_C,
    const FloatType rank_tol)
{
  // Number of observations.
  size_t m = T_C_W.size();
  CHECK_GE(m, 2u);

  // Compute unit-plane coorinates (divide by z) from bearing vectors.
  const Matrix2X uv = f_C.topRows<2>().array().rowwise() / f_C.bottomRows<1>().array();

  // Allocate DLT matrix.
  MatrixX4 A(m * 2, 4);
  A.setZero();

  // Fill DLT matrix.
  for (size_t i = 0; i < m; ++i)
  {
    //! @todo: Think if this can be optimized, e.g. without computing the Matrix44 and uv.
    size_t row = i * 2;
    Matrix44 projection = T_C_W[i].getTransformationMatrix();
    A.row(row)     = uv(0, i) * projection.row(2) - projection.row(0);
    A.row(row + 1) = uv(1, i) * projection.row(2) - projection.row(1);
  }
  int rank;
  FloatType error;
  VectorX v;
  std::tie(rank, error, v) = directLinearTransform(A, rank_tol);

  // Return homogeneous coordinates and success.
  Vector4 p_W_homogeneous = v;
  bool success = (rank < 3) ? false : true;
  return std::make_pair(p_W_homogeneous, success);
}

} // namespace ze
