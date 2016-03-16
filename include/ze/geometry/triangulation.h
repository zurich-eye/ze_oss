#pragma once

#include <tuple>

#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

//! Return depth in reference frame.
inline std::pair<FloatType, bool> depthFromTriangulation(
    const Transformation& T_cur_ref,
    const Eigen::Ref<const Bearing>& f_ref,
    const Eigen::Ref<const Bearing>& f_cur)
{
  Matrix32 A;
  A << T_cur_ref.getRotation().rotate(f_ref), f_cur;
  Matrix2 AtA = A.transpose() * A;
  if(AtA.determinant() < 0.000001)
  {
    return std::make_pair(0.0, false);
  }
  Vector2 depths = - AtA.inverse() * A.transpose() * T_cur_ref.getPosition();
  return std::make_pair(std::abs(depths(0)), true);
}

//! DLT triangulation [Hartley and Zisserman, 2nd edition, p. 312]
//! @param projection_matrices Projection matrices (K*P^-1)
//! @param uv_measurements 2D measurements on unit plane
//! @param rank_tol SVD rank tolerance
//! @return Triangulated point, in homogeneous coordinates
//! @return Success.
std::pair<Vector4, bool> triangulateHomogeneousDLT(
    const std::vector<Matrix34>& projection_matrices,
    const Matrix2X& uv_measurements, FloatType rank_tol);

} // namespace ze
