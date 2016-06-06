#include <ze/common/random.hpp>

namespace ze {

Vector3 randomDirection3D(bool deterministic)
{
  // equal-area projection according to:
  // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
  const FloatType z =
      sampleFromUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 - 1.0;
  const FloatType t =
      sampleFromUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 * M_PI;
  const FloatType r = std::sqrt(1.0 - z * z);
  const FloatType x = r * std::cos(t);
  const FloatType y = r * std::sin(t);
  return Vector3(x, y, z);
}

Vector2 randomDirection2D(bool deterministic)
{
  const FloatType theta =
      sampleFromUniformRealDistribution(deterministic, 0.0, 1.0) * 2.0 * M_PI;
  return Vector2(std::cos(theta), std::sin(theta));
}

} // namespace ze
