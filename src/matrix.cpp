// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/matrix.h>

#include <Eigen/SVD>
#include <Eigen/LU>

namespace ze {

std::tuple<int, real_t, VectorX> directLinearTransform(const MatrixX& A, real_t rank_tol)
{
  int n = A.rows();
  int p = A.cols();
  int m = std::min(n,p);
  Eigen::JacobiSVD<MatrixX> svd(A, Eigen::ComputeFullV);
  VectorX s = svd.singularValues();
  MatrixX V = svd.matrixV();

  // Find rank
  int rank = 0;
  for (int i = 0; i < m; ++i)
  {
    if (s(i) > rank_tol)
    {
      ++rank;
    }
  }

  // Return rank, error, and corresponding column of V
  real_t error = (m < p) ? 0.0 : s(m-1);
  return std::make_tuple(rank, error, V.col(p-1));
}

} // namespace ze
