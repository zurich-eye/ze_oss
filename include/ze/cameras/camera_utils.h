#pragma once

#include <ze/common/types.h>

namespace ze {

// Generate count random visible keypoints.
Keypoints generateRandomKeypoints(
    const int image_width, const int image_height, const int margin,
    const size_t count);

// Normalize a block of bearing vectors.
inline void normalizeBearings(
    Bearings& bearings)
{
  bearings = bearings.array().rowwise() / bearings.colwise().norm().array();
}

} // namespace ze
