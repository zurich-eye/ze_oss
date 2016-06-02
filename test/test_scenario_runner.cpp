#include <ze/imu_evaluation/scenario_runner.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/types.h>
#include <ze/imu_evaluation/imu_bias.hpp>
#include <ze/common/sampler.hpp>

TEST(ScenarioTest, testSplineScenario)
{
  using namespace ze;

  BSplinePoseMinimalRotationVector bs(3);

  bs.initPoseSpline(10.0, 20.0,
                    bs.curveValueToTransformation(Vector6::Random()),
                    bs.curveValueToTransformation(Vector6::Random()));

  SplineScenario::Ptr scenario(std::make_shared<SplineScenario>(bs));

  // dependencies:
  ImuBias::Ptr bias(std::make_shared<ConstantBias>());
  GaussianSampler<3>::Ptr acc_noise =
      GaussianSampler<3>::sigmas(Vector3(1e-5, 1e-5, 1e-5));
  GaussianSampler<3>::Ptr gyr_noise =
      GaussianSampler<3>::sigmas(Vector3(1e-5, 1e-5, 1e-5));
  Vector3 gravity(0, 0, -9.81);

  // test runner
  ScenarioRunner runner(scenario, bias, acc_noise, gyr_noise,
                        100, 100, gravity);

  for (FloatType t = 10.0; t < 20.0; t += 0.1)
  {
    // the probability is really high that this test passes... but not guaranteed
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
                  runner.angular_velocity_corrupted(t),
                  runner.angular_velocity_actual(t),
                  0.3));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
                  runner.specific_force_actual(t),
                  runner.specific_force_corrupted(t),
                  0.3));
  }

}

ZE_UNITTEST_ENTRYPOINT
