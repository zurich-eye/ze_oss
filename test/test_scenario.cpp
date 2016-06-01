#include <ze/imu/scenario.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/types.h>

TEST(ScenarioTest, testSplineScenario)
{
  using namespace ze;

  BSplinePoseMinimalRotationVector bs(3);

  bs.initPoseSpline(10.0, 20.0,
                    bs.curveValueToTransformation(Vector6::Random()),
                    bs.curveValueToTransformation(Vector6::Random()));

  SplineScenario scenario(bs);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs.transformation(11),
                                scenario.pose(11), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs.angularVelocityBodyFrame(11),
                                scenario.angular_velocity_body(11), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs.linearVelocity(11),
                                scenario.velocity(11), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs.linearAcceleration(11),
                                scenario.acceleration(11), 1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs.orientation(11),
                                scenario.orientation(11), 1e-8));
}

ZE_UNITTEST_ENTRYPOINT
