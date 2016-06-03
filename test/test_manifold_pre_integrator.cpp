#include <ze/imu_evaluation/manifold_pre_integrator.hpp>
#include <ze/common/transformation.h>
#include <ze/common/test_entrypoint.h>

TEST(ManifoldPreIntegratorTest, testPushD_R)
{
  using namespace ze;

  Vector3 covar_vector(1e-4, 1e-4, 1e-4);
  ManifoldPreIntegrationState state(covar_vector.asDiagonal());

  // Generate a set of random rotations.
  std::vector<Vector3> measurements;
  std::vector<FloatType> times;
  std::vector<FloatType> times2;
  int i;
  for (i = 0; i < 10; ++i)
  {
    times.push_back(i);
    times2.push_back(i + 11);
    measurements.push_back(Vector3::Random());
  }
  times.push_back(i);
  times2.push_back(i + 11);

  state.pushD_R(times, measurements);
  state.pushD_R(times2, measurements);
}

ZE_UNITTEST_ENTRYPOINT
