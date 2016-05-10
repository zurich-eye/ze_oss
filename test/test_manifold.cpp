#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_manifold.h>

TEST(ManifoldTests, testScalarTraits)
{
  EXPECT_EQ(ze::traits<ze::FloatType>::dimension, 1);
  ze::testManifoldInvariants<ze::FloatType>(1.0, 1.5);
  ze::testRetractJacobians<ze::FloatType>(1.0, 1.5);
  ze::testLocalJacobians<ze::FloatType>(1.0, 1.5);
}

TEST(ManifoldTests, testEigenTraits)
{
  EXPECT_EQ(ze::traits<ze::Vector3>::dimension, 3);
  ze::testManifoldInvariants<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
  ze::testRetractJacobians<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
  ze::testLocalJacobians<ze::Vector3>(
        ze::Vector3(1.0, 1.2, 1.3), ze::Vector3(2.0, 1.0, 0.0));
}

ZE_UNITTEST_ENTRYPOINT
