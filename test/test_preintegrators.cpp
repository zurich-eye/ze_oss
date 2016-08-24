// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/test_entrypoint.h>
#include <ze/common/types.h>
#include <ze/imu_evaluation/manifold_pre_integrator.hpp>
#include <ze/imu_evaluation/quaternion_pre_integrator.hpp>

TEST(PreIntegratorTest, testManifold)
{
  using namespace ze;

  Matrix3 covar = Matrix3::Identity() * 1e-4;

  PreIntegratorFactory::Ptr preintegrator_factory(
        std::make_shared<QuaternionPreIntegrationFactory>(covar,
                                     PreIntegrator::FirstOrderMidward));

  PreIntegrator::Ptr integrator = preintegrator_factory->get();

  int runs = 1000;

  std::vector<real_t> times(runs);
  std::iota(times.begin(), times.end(), 0);

  MatrixX values(6, runs);
  values.setRandom();

  integrator->pushD_R_i_j(times, values);

}

ZE_UNITTEST_ENTRYPOINT
