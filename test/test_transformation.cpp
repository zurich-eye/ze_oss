#include <cmath>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_manifold.h>
#include <ze/common/transformation.h>

#ifndef ZE_SINGLE_PRECISION_FLOAT
constexpr ze::FloatType tol = 1e-9;
#else
constexpr ze::FloatType tol = 1e-7;
#endif

TEST(TransformationTests, testManifoldSO3)
{
  EXPECT_EQ(ze::traits<ze::Quaternion>::dimension, 3);
  ze::testManifoldInvariants<ze::Quaternion>(
        ze::Quaternion(ze::Vector3(0.1, 0.2, 0.3)),
        ze::Quaternion(ze::Vector3(0.2, 0.3, 0.4)), tol);
#ifndef ZE_SINGLE_PRECISION_FLOAT
  ze::testRetractJacobians<ze::Quaternion>(
        ze::Quaternion(ze::Vector3(0.1, 0.2, 0.3)),
        ze::Quaternion(ze::Vector3(0.2, 0.3, 0.4)));
  ze::testLocalJacobians<ze::Quaternion>(
        ze::Quaternion(ze::Vector3(0.1, 0.2, 0.3)),
        ze::Quaternion(ze::Vector3(0.2, 0.3, 0.4)));
#else
  LOG(WARNING) << "Test ignored for single precision float.";
#endif
}

TEST(TransformationTests, testSetRandom)
{
  ze::Transformation T;
  T.setRandom();
  ze::Matrix3 R = T.getRotation().getRotationMatrix();

  // Check if orthonormal
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(R*R.transpose(), ze::I_3x3, 1e-6));
}

TEST(TransformationTests, testExpLog)
{
  for(int i = 0; i < 10; ++i)
  {
    ze::Transformation T1;
    T1.setRandom();
    ze::Transformation::Vector6 v = T1.log();
    ze::Transformation T2 = ze::Transformation::exp(v);
    ze::Matrix4 TT1 = T1.getTransformationMatrix();
    ze::Matrix4 TT2 = T2.getTransformationMatrix();
    for(int r = 0; r < 4; ++r)
    {
      for(int c = 0; c < 4; ++c)
      {
        EXPECT_NEAR(TT1(r,c), TT2(r,c), 1e-6) << "Failed at (" << r << "," << c << ")";
      }
    }
  }
}

TEST(TransformationTests, quaternionMatrices)
{
  Eigen::Quaterniond q_AB, q_BC, q_AC_plus, q_AC_oplus, q_AC_quatmult;
  q_AB.coeffs() = Eigen::Vector4d::Random().normalized();
  q_BC.coeffs() = Eigen::Vector4d::Random().normalized();
  q_AC_quatmult = q_AB * q_BC;
  q_AC_plus.coeffs() = ze::quaternionPlusMatrix(q_AB) * q_BC.coeffs();
  q_AC_oplus.coeffs() = ze::quaternionOplusMatrix(q_BC) * q_AB.coeffs();

  Eigen::Vector3d v_C = Eigen::Vector3d::Random();
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(q_AC_quatmult * v_C, q_AC_plus * v_C));
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(q_AC_quatmult * v_C, q_AC_oplus * v_C));
}

ZE_UNITTEST_ENTRYPOINT
