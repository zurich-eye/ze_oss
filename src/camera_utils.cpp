#include <ze/cameras/camera_utils.h>

#include <random>
#include <ze/common/logging.hpp>

#include <ze/cameras/camera_rig.h>

namespace ze {

// -----------------------------------------------------------------------------
Keypoints generateRandomKeypoints(
    const Size2u size,
    const uint32_t margin,
    const uint32_t num_keypoints)
{
  DEBUG_CHECK_GT(size.width(), margin + 1u);
  DEBUG_CHECK_GT(size.height(), margin + 1u);

  std::ranlux24 gen;
  std::uniform_real_distribution<FloatType> dist_x(margin, size.width() - 1 - margin);
  std::uniform_real_distribution<FloatType> dist_y(margin, size.height() - 1 - margin);

  Keypoints kp(2, num_keypoints);
  for(uint32_t i = 0u; i < num_keypoints; ++i)
  {
    kp(0,i) = dist_x(gen);
    kp(1,i) = dist_y(gen);
  }
  return kp;
}

// -----------------------------------------------------------------------------
Keypoints generateUniformKeypoints(
    const Size2u size,
    const uint32_t margin,
    const uint32_t num_cols)
{
  DEBUG_CHECK_GT(size.width(), margin + 1u);
  DEBUG_CHECK_GT(size.height(), margin + 1u);
  const uint32_t num_rows = num_cols * size.height() / size.width();

  // Compute width and height of a cell:
  FloatType w = (static_cast<FloatType>(size.width() - 0.01)  - 2.0 * margin) / (num_cols - 1);
  FloatType h = (static_cast<FloatType>(size.height() - 0.01) - 2.0 * margin) / (num_rows - 1);

  // Sample keypoints:
  Keypoints kp(2, num_rows * num_cols);
  for (uint32_t y = 0u; y < num_rows; ++y)
  {
    for (uint32_t x = 0u; x < num_cols; ++x)
    {
      uint32_t i = y * num_cols + x;
      kp(0,i) = margin + x * w;
      kp(1,i) = margin + y * h;
    }
  }
  return kp;
}

// -----------------------------------------------------------------------------
std::tuple<Keypoints, Bearings, Positions> generateRandomVisible3dPoints(
    const Camera& cam,
    const uint32_t num_points,
    const uint32_t margin,
    const FloatType min_depth,
    const FloatType max_depth)
{
  Keypoints px = generateRandomKeypoints(cam.size(), margin, num_points);
  Bearings f = cam.backProjectVectorized(px);
  Positions pos = f;
  std::ranlux24 gen;
  std::uniform_real_distribution<FloatType> scale(min_depth, max_depth);
  for(uint32_t i = 0u; i < num_points; ++i)
  {
    pos.col(i) *= scale(gen);
  }
  return std::make_tuple(px, f, pos);
}

// -----------------------------------------------------------------------------
FloatType overlappingFieldOfView(
    const CameraRig& rig,
    const uint32_t cam_A,
    const uint32_t cam_B)
{
  DEBUG_CHECK_LT(cam_A, rig.size());
  DEBUG_CHECK_LT(cam_B, rig.size());

  // We sample uniformly keypoints in camera a and project them into camera b,
  // assuming the landmark is at infinity (i.e. only rotate).
  Keypoints px_A = generateUniformKeypoints(rig.at(cam_A).size(), 0u, 20u);
  Bearings f_A = rig.at(cam_A).backProjectVectorized(px_A);
  Transformation T_B_A = rig.T_C_B(cam_B) * rig.T_C_B(cam_A).inverse();
  Positions p_B = T_B_A.getRotation().rotateVectorized(f_A);
  Keypoints px_B = rig.at(cam_B).projectVectorized(p_B);

  uint32_t num_visible = 0u;
  for (int i = 0; i < px_B.cols(); ++i)
  {
    //! @todo: Omnidirectional cameras: Improve check.
    if (p_B.col(i)(2) > 0 && isVisible(rig.at(cam_B).size(), px_B.col(i)))
    {
      ++num_visible;
    }
  }

  return static_cast<FloatType>(num_visible) / px_B.cols();
}

} // namespace ze
