#include <ze/common/sample.h>

namespace ze {

RandomGenerator::RandomGenerator()
  : gen_real_(std::random_device{}())
  , gen_int_(std::random_device{}())
  , gen_real_deterministic_(0)
  , gen_int_deterministic_(0)
{}

RandomGenerator& RandomGenerator::instance()
{
  static RandomGenerator gen;
  return gen;
}

std::ranlux24 RandomGenerator::generatorReal(bool deterministic)
{
  return deterministic ? instance().gen_real_deterministic_ : instance().gen_real_;
}

std::mt19937 RandomGenerator::generatorInt(bool deterministic)
{
  return deterministic ? instance().gen_int_deterministic_ : instance().gen_int_;
}

/*
int sampleFromUniformDistribution(int from, int to)
{
  std::uniform_int_distribution<int> distribution(from, to);
  return distribution(gen_int);
}

double Sample::uniform()
{
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  return distribution(gen_real);
}

double gaussian(double stddev)
{
  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(gen_real);
}

Vector3 randomDirection3D()
{
  // equal-area projection according to:
  // https://math.stackexchange.com/questions/44689/how-to-find-a-random-axis-or-unit-vector-in-3d
  const FloatType z = Sample::uniform() * 2.0 - 1.0;
  const FloatType t = Sample::uniform() * 2.0 * M_PI;
  const FloatType r = std::sqrt(1.0 - z*z);
  const FloatType x = r * std::cos(t);
  const FloatType y = r * std::sin(t);
  return Vector3(x,y,z);
}

Vector2 randomDirection2D()
{
  const double theta = Sample::uniform()*2.0*M_PI;
  return Eigen::Vector2d(std::cos(theta), std::sin(theta));
}
*/

} // namespace ze
