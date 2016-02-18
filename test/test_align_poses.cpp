#include <cmath>
#include <functional>
#include <random>
#include <utility>

#include <ze/common/numerical_derivative.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/geometry/align_poses.h>

TEST(AlignPosesTest, testJacobian)
{
  using namespace ze;

  Transformation T_A_B, T_W_A, T_W_B;
  T_A_B.setRandom(1.0);
  T_W_A.setRandom(1.0);
  T_W_B.setRandom(1.0);

  auto residualLambda = [&](const Transformation& T_A_B) {
      Vector3 pos = T_W_A * T_A_B.getPosition() - T_W_B.getPosition();
      Vector3 rot = Quaternion::log(T_W_B.getRotation().inverse()
                                    * (T_W_A.getRotation() * T_A_B.getRotation()));
      return (Vector6() << pos, rot).finished();
    };
  Matrix6 J_numeric = numericalDerivative<Vector6, Transformation>(residualLambda, T_A_B);
  Matrix6 J_analytic = dRelpose_dTransformation(T_A_B, T_W_A, T_W_B);

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
}

ZE_UNITTEST_ENTRYPOINT
