#pragma once

#include <ze/common/types.h>

namespace ze {

template<typename Scalar>
constexpr Scalar radToDeg(Scalar rad)
{
  return rad * 180.0 / M_PI;
}

template<typename Scalar>
constexpr Scalar degToRad(Scalar deg)
{
  return deg * M_PI / 180.0;
}

} // namespace ze
