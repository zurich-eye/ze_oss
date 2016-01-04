#include <ze/cameras/camera_utils.h>

#include <random>
#include <glog/logging.h>

namespace ze {

Keypoints generateRandomKeypoints(
    const int image_width, const int image_height, const int margin,
    const size_t count)
{
  CHECK_GE(image_width, 0);
  CHECK_GE(image_height, 0);

  std::ranlux24 gen;
  std::uniform_real_distribution<double> dist_x(margin, image_width - 1 - margin);
  std::uniform_real_distribution<double> dist_y(margin, image_height - 1 - margin);

  Keypoints kp(2, count);
  for(size_t i = 0; i < count; ++i)
  {
    kp(0,i) = dist_x(gen);
    kp(1,i) = dist_y(gen);
  }
  return kp;
}

} // namespace ze
