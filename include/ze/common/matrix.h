#pragma once

#include <ze/common/types.h>

namespace ze {

//! Skew symmetric matrix.
inline Matrix3 skewSymmetric(Vector3 w)
{
  return (Matrix3() <<
          0.0f, -w.z(), +w.y(),
          +w.z(), 0.0f, -w.x(),
          -w.y(), +w.x(), 0.0f).finished();
}

//! Normalize a block of bearing vectors.
inline void normalizeBearings(Bearings& bearings)
{
  bearings = bearings.array().rowwise() / bearings.colwise().norm().array();
}

//! Get element with max norm in a vector.
inline FloatType normMax(const VectorX& v)
{
  FloatType max = -1.0f;
  for(int i = 0; i < v.size(); ++i)
  {
    FloatType abs = std::abs(v[i]);
    if(abs > max)
    {
      max = abs;
    }
  }
  return max;
}

} // namespace ze
