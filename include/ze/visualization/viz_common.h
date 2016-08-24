// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.h>

namespace ze {

struct Color
{
  real_t r = 0.0f;
  real_t g = 0.0f;
  real_t b = 0.0f;
  real_t a = 1.0f;

  constexpr Color(real_t r, real_t g, real_t b)
    : r(r), g(g), b(b)
  {}

  constexpr Color(real_t r, real_t g, real_t b, real_t a)
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
  static constexpr Color White {1.0, 1.0, 1.0};
  static constexpr Color LightGray {0.8, 0.8, 0.8};
  static constexpr Color Yellow {1.0, 1.0, 0.0};
  static constexpr Color Magenta {1.0, 0.0, 1.0};
  static constexpr Color Orange {1.0, 0.5, 0.0};
};

} // namespace ze
