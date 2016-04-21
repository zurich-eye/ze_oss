#pragma once

#include <tuple>

#include <ze/common/types.h>
#include <imp/core/size.hpp>

namespace ze {

// fwd
class Camera;

//! Generate count random visible keypoints.
Keypoints generateRandomKeypoints(
    const uint32_t image_width, const uint32_t image_height,
    const uint32_t margin, const size_t count);

//! Generate random visible 3d points.
std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam, const size_t count,
    const int margin = 10, const FloatType min_depth = 1.0, const FloatType max_depth = 3.0);

//! Return if pixel u is within image boundaries.
template<typename DerivedKeyPoint>
bool isVisible(
    const Size2u image_size,
    const Eigen::MatrixBase<DerivedKeyPoint>& px)
{
  return px[0] >= 0u
      && px[1] >= 0u
      && px[0] <  image_size.width()
      && px[1] <  image_size.height();
}

//! Return if pixel u is within image boundaries.
template<typename DerivedKeyPoint>
bool isVisible(
    const typename DerivedKeyPoint::Scalar image_width,
    const typename DerivedKeyPoint::Scalar image_height,
    const Eigen::MatrixBase<DerivedKeyPoint>& px)
{
  return px[0] >= 0
      && px[1] >= 0
      && px[0] <  image_width
      && px[1] <  image_height;
}

//! Return if pixel px is within image boundaries with margin.
template<typename DerivedKeyPoint>
bool isVisibleWithMargin(
    const typename DerivedKeyPoint::Scalar image_width,
    const typename DerivedKeyPoint::Scalar image_height,
    const Eigen::MatrixBase<DerivedKeyPoint>& px,
    const typename DerivedKeyPoint::Scalar margin)
{
  return px[0] >= margin
      && px[1] >= margin
      && px[0] < (image_width - margin)
      && px[1] < (image_height - margin);
}

//! Return if pixel px is within image boundaries with margin.
inline bool isVisibleWithMargin(
    const int image_width, const int image_height, const int x, const int y, const int margin)
{
  return x >= margin
      && y >= margin
      && x < (image_width - margin)
      && y < (image_height - margin);
}

} // namespace ze
