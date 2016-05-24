#pragma once

#include <float.h>
#include <limits>
#include <map>
#include <string>
#include <ze/common/transformation.h>

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

template<class T>
typename std::enable_if<std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::default_random_engine generator;
  std::uniform_int_distribution<T> distribution(std::numeric_limits<T>::lowest(),
                                                std::numeric_limits<T>::max());
  auto random_val = std::bind(distribution, generator);
  return random_val;
}

template<class T>
typename std::enable_if<!std::is_integral<T>::value, std::function<T()> >::type
getRandomGenerator()
{
  std::random_device rd;
  std::mt19937 generator(rd());
  //std::default_random_engine generator;
  std::uniform_real_distribution<T> distribution(std::numeric_limits<T>::lowest(),
                                                 std::numeric_limits<T>::max());
  auto random_val = std::bind(distribution, std::ref(generator));
  return random_val;
}

} // namespace ze
