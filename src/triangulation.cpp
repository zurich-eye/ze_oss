#include <ze/geometry/triangulation.h>

#include <ze/common/matrix.h>

namespace ze {

std::pair<Vector4, bool> triangulateHomogeneousDLT(
    const std::vector<Matrix34>& projection_matrices,
    const Matrix2X& uv_measurements, FloatType rank_tol)
{
  // Number of observations.
  size_t m = projection_matrices.size();

  // Allocate DLT matrix.
  MatrixX4 A(m * 2, 4);
  A.setZero();

  // Fill DLT matrix.
  for (size_t i = 0; i < m; ++i)
  {
    size_t row = i * 2;
    const Matrix34& projection = projection_matrices[i];
    A.row(row)     = uv_measurements(0,i) * projection.row(2) - projection.row(0);
    A.row(row + 1) = uv_measurements(1,i) * projection.row(2) - projection.row(1);
  }
  int rank;
  FloatType error;
  VectorX v;
  std::tie(rank, error, v) = DLT(A, rank_tol);

  bool success = (rank < 3) ? false : true;
  return std::make_pair(Vector4(v), success);
}

} // namespace ze
