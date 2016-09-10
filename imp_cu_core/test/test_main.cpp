#include <gflags/gflags.h>
#include <gtest/gtest.h>
#include <cuda_runtime_api.h>
#include <ze/common/logging.hpp>

/// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  int ret = RUN_ALL_TESTS();
  cudaDeviceReset();
  return ret;
}
