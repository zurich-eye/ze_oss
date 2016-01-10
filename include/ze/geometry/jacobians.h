#pragma once

#include <ze/common/types.h>

namespace ze {

/*!
 * @brief Jacobian of bearing vector w.r.t. landmark in camera coordinates.
 *
 * f = norm(p) = p / sqrt(x^2 + y^2 + z^2), with p = [x, y, z].
 *
 *                                       | y^2 + z^2, -xy, -xz |
 * df/dp = 1 / (x^2 + y^2 + z^2)^(3/2) * | -xy, x^2 + z^2, -yz |
 *                                       | -xz, -yz, x^2 + z^2 |.
 */
inline Eigen::Matrix3d dBearing_dLandmark(const Position& p_C)
{
  double x2 = p_C(0) * p_C(0);
  double y2 = p_C(1) * p_C(1);
  double z2 = p_C(2) * p_C(2);
  double xy = p_C(0) * p_C(1);
  double xz = p_C(0) * p_C(2);
  double yz = p_C(1) * p_C(2);
  double x2_y2_z2 = x2 + y2 + z2;
  Eigen::Matrix3d J;
  J << y2 + z2, -xy, -xz,
       -xy, x2 + z2, -yz,
       -xz, -yz, x2 + y2;
  J /= (x2_y2_z2 * std::sqrt(x2_y2_z2));
  return J;
}

} // namespace ze
