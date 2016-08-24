// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/benchmark.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/random_matrix.hpp>
#include <ze/common/running_statistics.h>

TEST(RandomMatrixTests, testRandomVectorSampler)
{
  using namespace ze;

  Vector2 var_vector;
  var_vector << 2, 3;
  RandomVectorSampler<2>::Ptr sampler = RandomVectorSampler<2>::variances(var_vector, true);
  RandomVectorSampler<3>::Ptr sampler2 = RandomVectorSampler<3>::sigmas(Vector3(1, 2, 3));

  Vector2 sample = sampler->sample();
  Vector3 sample2 = sampler2->sample();
}

TEST(RandomMatrixTests, testRandomVector)
{
  using namespace ze;

  VLOG(1) << "Deterministic:";
  Vector2 v1 = randomVectorUniformDistributed<2>(true);
  EXPECT_NEAR(v1(0), 0.592845, 1e-5);
  EXPECT_NEAR(v1(1), 0.844266, 1e-5);
  Vector2 v2 = randomVectorUniformDistributed<2>(true);
  EXPECT_NEAR(v2(0), 0.857946, 1e-5);
  EXPECT_NEAR(v2(1), 0.847252, 1e-5);

  VLOG(1) << "\n" << randomMatrixUniformDistributed<2,3>(true);
  VLOG(1) << "\n" << randomMatrixNormalDistributed<2,3>(true);
  VLOG(1) << "\n" << randomVectorNormalDistributed<4>(true).transpose();
  VLOG(1) << "\n" << randomMatrixNormalDistributed(2, 3, true);
  VLOG(1) << "Nondeterministic:";
  VLOG(1) << "\n" << randomMatrixUniformDistributed<2,3>();
  VLOG(1) << "\n" << randomMatrixNormalDistributed<2,3>();
  VLOG(1) << "\n" << randomVectorNormalDistributed<4>().transpose();
  VLOG(1) << "\n" << randomMatrixNormalDistributed(2, 3);
}

ZE_UNITTEST_ENTRYPOINT
