#include <ze/geometry/align_points.h>

#include <cmath>
#include <numeric>
#include <algorithm>
#include <ze/common/logging.hpp>

#include <ze/common/matrix.h>
#include <ze/common/stl_utils.h>

namespace ze {

PointAligner::PointAligner(
    const Positions& p_A,
    const Positions& p_B)
  : p_A_(p_A)
  , p_B_(p_B)
{
  CHECK_EQ(p_A_.cols(), p_B_.cols());
}

double PointAligner::evaluateError(
    const Transformation& T_A_B,
    HessianMatrix* H,
    GradientVector* g)
{
  double chi2 = 0.0;

  // Compute prediction error.
  Positions residuals = p_A_ - T_A_B.transformVectorized(p_B_);
  VectorX residuals_norm = residuals.colwise().norm();

  // At the first iteration, compute the scale of the error.
  if(iter_ == 0)
  {
    measurement_sigma_ = ScaleEstimator::compute(residuals_norm);
    VLOG(1) << "measurement sigma = " << measurement_sigma_;
  }

  // Robust cost function.
  VectorX weights =
      WeightFunction::weightVectorized(residuals_norm / measurement_sigma_);

  // Whiten error.
  residuals_norm /= measurement_sigma_;

  if(H && g)
  {
    for(int i = 0; i < p_A_.cols(); ++i)
    {
      // Compute Jacobian (if necessary, this can be optimized a lot).
      Matrix36 J = dPointdistance_dRelpose(T_A_B, p_A_.col(i), p_B_.col(i));

      // Whiten Jacobian.
      J /= measurement_sigma_;

      // Compute Hessian and Gradient Vector.
      H->noalias() += J.transpose() * J * weights(i);
      g->noalias() -= J.transpose() * residuals.col(i) * weights(i);
    }
  }

  // Compute log-likelihood : 1/(2*sigma^2)*(z-h(x))^2 = 1/2*e'R'*R*e
  chi2 += 0.5 * weights.dot(residuals.colwise().squaredNorm());

  return chi2;
}

Transformation PointAligner::alignSE3(
    const Positions& pts_A, const Positions& pts_B)
{
  CHECK_GE(pts_A.cols(), 0u);
  CHECK_EQ(pts_A.cols(), pts_B.cols());

  // Compute T_B_A using the method by K. S. Arun et al.:
  // Least-Squares Fitting of Two 3-D Point Sets
  // IEEE Trans. Pattern Anal. Mach. Intell., 9, NO. 5, SEPTEMBER 1987
  const Vector3 mean_pts_A = pts_A.rowwise().mean();
  const Vector3 mean_pts_B = pts_B.rowwise().mean();
  const Positions zero_centered_A = pts_A.colwise() - mean_pts_A;
  const Positions zero_centered_B = pts_B.colwise() - mean_pts_B;
  const FloatType n = static_cast<FloatType>(pts_A.cols());
  const Matrix3 sigma =
      zero_centered_A * zero_centered_B.transpose() / n;

  Eigen::JacobiSVD<Matrix3> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int rank = svd.rank();
  CHECK_GE(rank, 2);
  const Matrix3 svd_U = svd.matrixU();
  const Matrix3 svd_V = svd.matrixV();
  Matrix3 svd_S = Matrix3::Identity();
  if ((rank > 2 && sigma.determinant() < 0.0) ||
      (rank == 2 && svd_U.determinant() * svd_V.determinant() < 0.0))
  {
    svd_S(2, 2) = -1.0;
  }
  const Matrix3 R_A_B = svd_U * svd_S * svd_V.transpose();
  const Position t_A_B = mean_pts_A - R_A_B * mean_pts_B;
  return Transformation(t_A_B, Quaternion(R_A_B)).inverse();
}

std::pair<FloatType, Transformation> PointAligner::alignSim3(
    const Positions& pts_A, const Positions& pts_B)
{
  CHECK_NE(pts_A.cols(), 0u);
  CHECK_EQ(pts_A.cols(), pts_B.cols());

  // Compute <s, T_B_A> using the method by
  // S. Umeyama: Least-Squares Estimation
  // of Transformation Parameters Between Two Point Patterns,
  // IEEE Trans. Pattern Anal. Mach. Intell., vol. 13, no. 4, 1991.
  const Vector3 mean_pts_A = pts_A.rowwise().mean();
  const Vector3 mean_pts_B = pts_B.rowwise().mean();
  Positions zero_centered_A = pts_A;
  Positions zero_centered_B = pts_B;
  zero_centered_A.colwise() -= mean_pts_A;
  zero_centered_B.colwise() -= mean_pts_B;

  const FloatType n = static_cast<FloatType>(pts_A.cols());
  const Matrix3 c = zero_centered_A * zero_centered_B.transpose() / n;
  const FloatType sigma2 = 1.0 / n * (zero_centered_B.array() * zero_centered_B.array()).sum();

  Eigen::JacobiSVD<Matrix3> svd(c, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const int rank = svd.rank();
  CHECK_GE(rank, 2);
  const Matrix3 svd_U = svd.matrixU();
  const Matrix3 svd_V = svd.matrixV();
  const Matrix3 svd_D = svd.singularValues().asDiagonal();
  Matrix3 S = Matrix3::Identity();
  if ((rank > 2 && c.determinant() < 0.0) ||
      (rank == 2 && svd_U.determinant() * svd_V.determinant() < 0.0))
  {
    S(2, 2) = -1.0;
  }

  const Matrix3 R_A_B = svd_U * S * svd_V;
  const FloatType s = 1.0 / sigma2 * (svd_D * S).trace();
  const Vector3 t_A_B = mean_pts_A - s * R_A_B * mean_pts_B;
  return std::pair<FloatType, Transformation>(
        1.0/s, Transformation(t_A_B, Quaternion(R_A_B)).inverse());
}

} // namespace ze


