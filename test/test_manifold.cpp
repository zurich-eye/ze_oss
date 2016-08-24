// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_manifold.h>

TEST(ManifoldTests, testScalarTraits)
{
  EXPECT_EQ(ze::traits<ze::real_t>::dimension, 1);
  ze::testManifoldInvariants<ze::real_t>(1.0, 1.5);
#ifndef ZE_SINGLE_PRECISION_FLOAT
  ze::testRetractJacobians<ze::real_t>(1.0, 1.5);
  ze::testLocalJacobians<ze::real_t>(1.0, 1.5);
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

TEST(ManifoldTests, testEigenTraits)
{
  EXPECT_EQ(ze::traits<ze::Vector3>::dimension, 3);
  ze::testManifoldInvariants<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
#ifndef ZE_SINGLE_PRECISION_FLOAT
  ze::testRetractJacobians<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
  ze::testLocalJacobians<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
#else
  LOG(WARNING) << "Numerical derivative test ignored for single precision float.";
#endif
}

ZE_UNITTEST_ENTRYPOINT
