#pragma once

#include <random>
#include <ze/common/types.h>
#include <ze/common/noncopyable.hpp>


namespace ze {

//! Get deterministic or non-deterministc random number generator. Singleton.
class RandomGenerator : Noncopyable
{
public:
  static std::ranlux24 generatorReal(bool deterministic = false);
  static std::mt19937  generatorInt(bool deterministic = false);

private:
  RandomGenerator();

  static RandomGenerator& instance();

  std::ranlux24 gen_real_;
  std::mt19937 gen_int_;
  std::ranlux24 gen_real_deterministic_;
  std::mt19937 gen_int_deterministic_;
};


template<class T>
typename std::enable_if<std::is_integral<T>::value, std::function<T()> >::type
sampleFromUniformDistribution(
    T from = std::numeric_limits<T>::lowest(),
    T to   = std::numeric_limits<T>::max(),
    bool deterministic = false)
{
  auto gen = RandomGenerator::generatorInt(deterministic);
  auto dist = std::uniform_int_distribution<T>(from, to);
  return dist(gen);
}

/*

FloatType sampleFromUniformDistribution();

double gaussian(double sigma);

//! @return 3-dimensional unit vector.
Vector3 randomDirection3D();

//! @return 2-dimensional unit vector.
Vector2 randomDirection2D();

// ----------------------------------------------------------------------------
//! Random number generator for integral types.
//! @return Random number in interval [lowest, highest]
template<class T>
typename std::enable_if<std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::mt19937 generator(std::random_device{}());
  std::uniform_int_distribution<T> distribution(std::numeric_limits<T>::lowest(),
                                                std::numeric_limits<T>::max());
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

// ----------------------------------------------------------------------------
//! Random number generator for real types.
//! @return Random number in interval [0.0, 1.0]
template<class T>
typename std::enable_if<!std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::mt19937 generator(std::random_device{}());
  //! @todo (MWE) enable a min/max rnd generator - not working yet so using [0.0,1.0] for now
  std::uniform_real_distribution<T> distribution(0.0, 1.0);
//  std::numeric_limits<T>::lowest(),
//      std::numeric_limits<T>::max());
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

// ----------------------------------------------------------------------------
//! Random number generator that returns a FloatType between 0.0 and 1.0.
inline std::function<FloatType()> getRandomGenerator01()
{
  std::mt19937 generator(std::random_device{}());
  std::uniform_real_distribution<FloatType> distribution(FloatType{0.0}, FloatType{1.0});
  std::function<FloatType()> random_val = std::bind(distribution, generator);
  return random_val;
}
*/

} // namespace ze
