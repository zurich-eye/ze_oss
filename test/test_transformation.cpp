#include <cmath>

#include <ze/common/test/entrypoint.h>
#include <ze/common/transformation.h>

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
    for(int r = 0; r < 4; ++r) {
      for(int c = 0; c < 4; ++c) {
        EXPECT_NEAR(TT1(r,c), TT2(r,c), 1e-6) << "Failed at (" << r << "," << c << ")";
      }
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
