#pragma once

#include <float.h>
#include <limits>
#include <map>
#include <string>
#include <ze/common/transformation.h>
#include <ze/common/types.h>

namespace ze {

//! Get path to the "data" folder in the ze_test_data repository. Therefore,
//! the environment variable ZE_TEST_DATA_PATH must be set.
std::string getTestDataDir(const std::string& dataset_name);

//! Load poses from .csv file. Returns a map { Image-Index / Stamp -> Pose }
std::map<int64_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename);

//! Load depthmap from .depth file. Copy into raw datapointer of size data_size
//! because ze_common does not depend on imp_core. data must be preallocated!
void loadDepthmapFromFile(
    const std::string& filename, const size_t data_size, float* data);

// ############################################################################
// RANDOM NUMBER GENERATORS

// ----------------------------------------------------------------------------
//! Random number generator for integral types.
//! @return Random number in interval [lowest, highest]
template<class T>
inline
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
inline
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

// ----------------------------------------------------------------------------
//! Bernoulli distribution, returns true with probability `true_probability` and
//! false with probability `1-true_probability`
inline std::function<bool()> getRandomGeneratorBinary(FloatType true_probability)
{
  CHECK_GE(true_probability, FloatType{0.0});
  CHECK_LE(true_probability, FloatType{1.0});
  std::mt19937 generator(std::random_device{}());
  std::bernoulli_distribution distribution(true_probability);
  std::function<bool()> random_val = std::bind(distribution, generator);
  return random_val;
}

} // namespace ze
