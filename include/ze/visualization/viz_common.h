#pragma once

#include <ze/common/types.h>

namespace ze {

struct Color
{
  FloatType r = 0.0f;
  FloatType g = 0.0f;
  FloatType b = 0.0f;
  FloatType a = 1.0f;

  constexpr Color(FloatType r, FloatType g, FloatType b)
    : r(r), g(g), b(b)
  {}

  constexpr Color(FloatType r, FloatType g, FloatType b, FloatType a)
    : r(r), g(g), b(b), a(a)
  {}
};

struct Colors
{
  static constexpr Color Red {1.0, 0.0, 0.0};
  static constexpr Color Green {0.0, 1.0, 0.0};
  static constexpr Color Blue {0.0, 0.0, 1.0};
  static constexpr Color DarkRed {0.5, 0.0, 0.0};
  static constexpr Color DarkGreen {0.0, 0.5, 0.0};
  static constexpr Color DarkBlue {0.0, 0.0, 0.5};
};

} // namespace ze
