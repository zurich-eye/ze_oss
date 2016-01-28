#pragma once

#include <ze/common/types.h>
#include <ze/common/transformation.h>

namespace ze {

inline std::pair<FloatType, bool> depthFromTriangulation(
    const Transformation& T_cur_ref,
    const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur)
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

} // namespace ze
