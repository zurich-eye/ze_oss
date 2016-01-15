#pragma once

#include <map>
#include <string>
#include <ze/common/transformation.h>

namespace ze {

//! Get path to the "data" folder in the ze_test_data repository. Therefore,
//! the environment variable ZE_TEST_DATA_PATH must be set.
std::string getTestDataDir(const std::string& dataset_name);

//! Load poses from .csv file.
std::map<size_t, Transformation> loadIndexedPosesFromCsv(const std::string& filename);

//! Load depthmap from .depth file. Copy into raw datapointer of size data_size
//! because ze_common does not depend on imp_core.
void loadDepthmapFromFile(
    const std::string& filename, float* data, const size_t data_size);

} // namespace ze
