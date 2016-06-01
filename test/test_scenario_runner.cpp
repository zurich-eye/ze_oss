#include <ze/imu/scenario_runner.h>
#include <ze/common/test_entrypoint.h>
#include <ze/splines/bspline_pose_minimal.h>
#include <ze/common/types.h>
#include <ze/imu/imu_bias.h>
#include <ze/common/sampler.h>

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
  GaussianSampler<3>::Ptr acc_noise(
        std::make_shared<GaussianSampler<3>>(Vector3(1e-5, 1e-5, 1e-5)));
  GaussianSampler<3>::Ptr gyr_noise(
        std::make_shared<GaussianSampler<3>>(Vector3(1e-5, 1e-5, 1e-5)));
  Vector3 gravity(0, 0, -9.81);

  // test runner
  ScenarioRunner runner(scenario, bias, acc_noise, gyr_noise, gravity);

  for (FloatType t = 10.0; t < 20.0; t += 0.1)
  {
    // the probability is really high that this test passes... but not guaranteed
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
                  runner.angular_velocity_corrupted(t),
                  runner.angular_velocity_actual(t),
                  1e-3));
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(
                  runner.acceleration_corrupted(t),
                  runner.acceleration_actual(t),
                  1e-3));
  }

}

ZE_UNITTEST_ENTRYPOINT
