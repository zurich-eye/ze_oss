// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/vi_simulation/imu_simulator.hpp>
#include <ze/common/test_entrypoint.hpp>
#include <ze/splines/bspline_pose_minimal.hpp>
#include <ze/common/types.hpp>

TEST(ImuSimulator, testSplineScenario)
{
  using namespace ze;

  std::shared_ptr<BSplinePoseMinimalRotationVector> bs =
      std::make_shared<BSplinePoseMinimalRotationVector>(3);

  bs->initPoseSpline(10.0, 20.0,
                     bs->curveValueToTransformation(Vector6::Random()),
                     bs->curveValueToTransformation(Vector6::Random()));

  SplineTrajectorySimulator scenario(bs);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs->transformation(11),
                                scenario.T_W_B(11).getTransformationMatrix(),
                                1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs->angularVelocityBodyFrame(11),
                                scenario.angularVelocity_B(11),
                                1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs->linearVelocity(11),
                                scenario.velocity_W(11),
                                1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs->linearAcceleration(11),
                                scenario.acceleration_W(11),
                                1e-8));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(bs->orientation(11),
                                scenario.R_W_B(11).getRotationMatrix(),
                                1e-8));
}

ZE_UNITTEST_ENTRYPOINT
