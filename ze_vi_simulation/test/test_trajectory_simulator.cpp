// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
