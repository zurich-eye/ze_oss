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
