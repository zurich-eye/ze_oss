#pragma once

#include <ze/common/types.h>

namespace ze {

//! Skew symmetric matrix.
inline Matrix3 skewSymmetric(const Eigen::Ref<const Vector3>& w)
{
  return (Matrix3() <<
           0.0f, -w(2),  w(1),
           w(2),  0.0f, -w(0),
          -w(1),  w(0),  0.0f).finished();
}

//! Normalize a block of bearing vectors.
inline void normalizeBearings(Bearings& bearings)
{
  bearings = bearings.array().rowwise() / bearings.colwise().norm().array();
}

//! Get element with max norm in a vector.
inline FloatType normMax(const VectorX& v)
{
  return v.lpNorm<Eigen::Infinity>();
}

} // namespace ze
