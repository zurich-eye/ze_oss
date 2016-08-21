#pragma once

#include <tuple>

#include <ze/common/types.h>
#include <imp/core/size.hpp>

namespace ze {

// fwd
class Camera;
class CameraRig;

// -----------------------------------------------------------------------------
// Generate visible keypoints and landmarks.

//! Generate random visible keypoints.
Keypoints generateRandomKeypoints(
    const Size2u image_size,
    const uint32_t margin,
    const uint32_t num_keypoints);

//! Generate count random visible keypoints.
Keypoints generateUniformKeypoints(
    const Size2u image_size,
    const uint32_t margin,
    const uint32_t num_keypoints);

//! Generate random visible 3d points.
std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam,
    const uint32_t num_points,
    const uint32_t margin = 10u,
    const real_t min_depth = 1.0,
    const real_t max_depth = 3.0);

// -----------------------------------------------------------------------------
// Check overlapping field of view.

//! Check if two cameras in a rig have an overlapping field of view.
//! @return Approximate percentage of overlapping field of view between cameras.
real_t overlappingFieldOfView(
    const CameraRig& rig,
    const uint32_t cam_a,
    const uint32_t cam_b);

// -----------------------------------------------------------------------------
// Check landmark visiblity.

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
template<typename DerivedKeyPoint>
bool isVisibleWithMargin(
    const Size2u image_size,
    const Eigen::MatrixBase<DerivedKeyPoint>& px,
    const typename DerivedKeyPoint::Scalar margin)
{
  return px[0] >= margin
      && px[1] >= margin
      && px[0] < (static_cast<typename DerivedKeyPoint::Scalar>(image_size.width()) - margin)
      && px[1] < (static_cast<typename DerivedKeyPoint::Scalar>(image_size.height()) - margin);
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
