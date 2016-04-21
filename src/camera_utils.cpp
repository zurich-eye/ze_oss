#include <ze/cameras/camera_utils.h>

#include <random>
#include <ze/common/logging.hpp>

#include <ze/cameras/camera.h>

namespace ze {

Keypoints generateRandomKeypoints(
    const uint32_t image_width, const uint32_t image_height, const uint32_t margin, const size_t count)
{
  CHECK_GT(image_width, margin);
  CHECK_GT(image_height, margin);

  std::ranlux24 gen;
  std::uniform_real_distribution<FloatType> dist_x(margin, image_width - 1 - margin);
  std::uniform_real_distribution<FloatType> dist_y(margin, image_height - 1 - margin);

  Keypoints kp(2, count);
  for(size_t i = 0; i < count; ++i)
  {
    kp(0,i) = dist_x(gen);
    kp(1,i) = dist_y(gen);
  }
  return kp;
}

std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam, const size_t count,
    const int margin, const FloatType min_depth, const FloatType max_depth)
{
  Keypoints px = generateRandomKeypoints(cam.width(), cam.height(), margin, count);
  Bearings f = cam.backProjectVectorized(px);
  Positions pos = f;
  std::ranlux24 gen;
  std::uniform_real_distribution<FloatType> scale(min_depth, max_depth);
  for(size_t i = 0u; i < count; ++i)
  {
    pos.col(i) *= scale(gen);
  }
  return std::make_tuple(px, f, pos);
}

} // namespace ze
