#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_manifold.h>

TEST(ManifoldTests, testScalarTraits)
{
  EXPECT_EQ(ze::traits<double>::dimension, 1);
  ze::testManifoldInvariants<double>(1.0, 1.5);
  ze::testRetractJacobians<double>(1.0, 1.5);
  ze::testLocalJacobians<double>(1.0, 1.5);
}

TEST(ManifoldTests, testEigenTraits)
{
  EXPECT_EQ(ze::traits<Eigen::Vector3d>::dimension, 3);
  ze::testManifoldInvariants<Eigen::Vector3d>(
        Eigen::Vector3d(1.0, 1.2, 1.3), Eigen::Vector3d(2.0, 1.0, 0.0));
  ze::testRetractJacobians<Eigen::Vector3d>(
        Eigen::Vector3d(1.0, 1.2, 1.3), Eigen::Vector3d(2.0, 1.0, 0.0));
  ze::testLocalJacobians<Eigen::Vector3d>(
        Eigen::Vector3d(1.0, 1.2, 1.3), Eigen::Vector3d(2.0, 1.0, 0.0));
}

ZE_UNITTEST_ENTRYPOINT
