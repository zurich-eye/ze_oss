#pragma once

#include <Eigen/Core>
#include <ze/common/types.h>

namespace ze {

//! Skew symmetric matrix.
Eigen::Matrix3d skewSymmetric(Eigen::Vector3d w)
{
  return (Eigen::Matrix3d() <<
          0.0, -w.z(), +w.y(),
          +w.z(), 0.0, -w.x(),
          -w.y(), +w.x(), 0.0).finished();
}

//! Normalize a block of bearing vectors.
inline void normalizeBearings(Bearings& bearings)
{
  bearings = bearings.array().rowwise() / bearings.colwise().norm().array();
}

} // namespace ze
