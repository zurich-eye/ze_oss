#include <cmath>

#include <ze/common/test/entrypoint.h>
#include <ze/common/test/manifold.h>
#include <ze/common/transformation.h>

TEST(TransformationTests, testManifoldSO3)
{
  EXPECT_EQ(ze::traits<ze::Quaternion>::dimension, 3);
  ze::testManifoldInvariants<ze::Quaternion>(
        ze::Quaternion(Eigen::Vector3d(0.1, 0.2, 0.3)),
        ze::Quaternion(Eigen::Vector3d(0.2, 0.3, 0.4)));
  ze::testRetractJacobians<ze::Quaternion>(
        ze::Quaternion(Eigen::Vector3d(0.1, 0.2, 0.3)),
        ze::Quaternion(Eigen::Vector3d(0.2, 0.3, 0.4)));
  ze::testLocalJacobians<ze::Quaternion>(
        ze::Quaternion(Eigen::Vector3d(0.1, 0.2, 0.3)),
        ze::Quaternion(Eigen::Vector3d(0.2, 0.3, 0.4)));
}

TEST(TransformationTests, testSetRandom)
{
  ze::Transformation T;
  T.setRandom();
  Eigen::Matrix3d R = T.getRotation().getRotationMatrix();

  // Check if orthonormal
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(R*R.transpose(), Eigen::Matrix3d::Identity(), 1e-6));
}

TEST(TransformationTests, testExpLog)
{
  for(int i = 0; i < 10; ++i)
  {
    ze::Transformation T1;
    T1.setRandom();
    ze::Transformation::Vector6 v = T1.log();
    ze::Transformation T2 = ze::Transformation::exp(v);
    Eigen::Matrix4d TT1 = T1.getTransformationMatrix();
    Eigen::Matrix4d TT2 = T2.getTransformationMatrix();
    for(int r = 0; r < 4; ++r)
    {
      for(int c = 0; c < 4; ++c)
      {
        EXPECT_NEAR(TT1(r,c), TT2(r,c), 1e-6) << "Failed at (" << r << "," << c << ")";
      }
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
