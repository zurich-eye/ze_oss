#pragma once

#include <ze/common/types.h>

namespace ze {

//! Generate count random visible keypoints.
Keypoints generateRandomKeypoints(
    const int image_width, const int image_height, const int margin, const size_t count);

} // namespace ze
