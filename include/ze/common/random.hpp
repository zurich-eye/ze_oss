#pragma once

#include <random>
#include <ze/common/logging.hpp>
#include <ze/common/types.h>

namespace ze {

//------------------------------------------------------------------------------
//! @return Sample from integer-valued distribution.
template<typename T>
T sampleFromUniformIntDistribution(
    bool deterministic = false,
    T from = std::numeric_limits<T>::lowest(),
    T to   = std::numeric_limits<T>::max())
{
  static std::mt19937 gen_nondeterministic(std::random_device{}());
  static std::mt19937 gen_deterministic(0);
  auto dist = std::uniform_int_distribution<T>(from, to);
  return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}

//------------------------------------------------------------------------------
//! @return Sample from uniform real-valued distribution.
template<typename T>
T sampleFromUniformRealDistribution(
    bool deterministic = false,
    T from = T{0.0},
    T to   = T{1.0})
{
  static std::ranlux24 gen_nondeterministic(std::random_device{}());
  static std::ranlux24 gen_deterministic(0);
  auto dist = std::uniform_real_distribution<T>(from, to);
  return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}

//------------------------------------------------------------------------------
//! @return Sample from normal distribution (real-valued).
template<typename T>
T sampleFromNormalDistribution(
    bool deterministic = false,
    T mean = T{0.0},
    T sigma   = T{1.0})
{
  static std::ranlux24 gen_nondeterministic(std::random_device{}());
  static std::ranlux24 gen_deterministic(0);
  auto dist = std::normal_distribution<T>(mean, sigma);
  return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}

//------------------------------------------------------------------------------
//! @return true with given probability. Samples the Bernoulli distribution.
bool flipCoin(
    bool deterministic = false,
    FloatType true_probability = FloatType{0.5})
{
  DEBUG_CHECK_GE(true_probability, 0.0);
  DEBUG_CHECK_LT(true_probability, 1.0);
  static std::mt19937 gen_nondeterministic(std::random_device{}());
  static std::mt19937 gen_deterministic(0);
  auto dist = std::bernoulli_distribution(true_probability);
  return deterministic ? dist(gen_deterministic) : dist(gen_nondeterministic);
}

//------------------------------------------------------------------------------
// Sample manifolds:

//! @return 3-dimensional unit vector.
Vector3 randomDirection3D();

//! @return 2-dimensional unit vector.
Vector2 randomDirection2D();


// ----------------------------------------------------------------------------
//! Usage: f = uniformIntDistribution(); sample = f();
//! @return Uniform integer distribution in interval [from, to].
template<class T>
std::function<T()> uniformIntDistribution(
    bool deterministic = false,
    T from = std::numeric_limits<T>::lowest(),
    T to   = std::numeric_limits<T>::max())
{
  static std::mt19937 gen_nondeterministic(std::random_device{}());
  static std::mt19937 gen_deterministic(0);
  std::uniform_int_distribution<T> distribution(from, to);
  auto fun = deterministic ?
               std::bind(distribution, gen_deterministic) :
               std::bind(distribution, gen_nondeterministic);
  return fun;
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

} // namespace ze
