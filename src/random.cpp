#include <ze/common/random.hpp>

namespace ze {

Vector3 randomDirection3D(bool deterministic)
{
  // equal-area projection according to:
  // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
  const real_t z =
      sampleUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 - 1.0;
  const real_t t =
      sampleUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 * M_PI;
  const real_t r = std::sqrt(1.0 - z * z);
  const real_t x = r * std::cos(t);
  const real_t y = r * std::sin(t);
  return Vector3(x, y, z);
}

Vector2 randomDirection2D(bool deterministic)
{
  const real_t theta =
      sampleUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 * M_PI;
  return Vector2(std::cos(theta), std::sin(theta));
}

} // namespace ze
