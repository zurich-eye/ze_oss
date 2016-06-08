#include <functional>

#include <ze/common/line.hpp>
#include <ze/common/numerical_derivative.h>
#include <ze/common/test_entrypoint.h>

TEST(LineTests, testLineGeneration)
{
  using namespace ze;
  // Create lines.
  size_t n = 100;
  ze::Lines lines;
  ze::Positions start, end;
  generateRandomLines(n, lines, &start, &end);

  for (size_t i = 0; i < n; ++i)
  {
    Vector3 direction = lines[i].direction();
    Position anchor_point = lines[i].anchorPoint();
    // Check whether anchor point is really the closest point available.
    EXPECT_NEAR(direction.dot(anchor_point), 0.0, 1e-10);
    // Direction should be a normalized vector.
    EXPECT_NEAR(direction.norm(), 1.0, 1e-15);
    // Anchor, start and end point should be on the line.
    EXPECT_NEAR(lines[i].calculateDistanceToLine(anchor_point), 0.0, 1e-14);
    EXPECT_NEAR(lines[i].calculateDistanceToLine(start.col(i)), 0.0, 1e-14);
    EXPECT_NEAR(lines[i].calculateDistanceToLine(end.col(i)), 0.0, 1e-14);
  }
}

TEST(LineTests, testJacobian)
{
  using namespace ze;
  Transformation T_B_W, T_C_B;
  T_B_W.setRandom();
  T_C_B.setRandom();
  const Transformation T_C_W = T_C_B * T_B_W;

  Lines lines;
  Positions start, end;
  generateRandomLines(1, lines, &start, &end);

  const LineMeasurement n = (T_C_W * start.col(0)).cross(T_C_W * end.col(0));

  auto measurementError = [&](const Transformation& T_B_W) {
    Transformation T_C_W = T_C_B * T_B_W;
    Position camera_pos_W = T_C_W.inverse().getPosition();
    Vector2 error;
    error(0) = T_C_W.getRotation().inverse().rotate(n).dot(lines[0].direction());
    error(1) =
        T_C_W.getRotation().inverse().rotate(n).dot(camera_pos_W - lines[0].anchorPoint()) /
        (camera_pos_W - lines[0].anchorPoint()).norm();
    return error;
  };

  Matrix26 J_numeric = numericalDerivative<Vector2, Transformation>(measurementError, T_B_W);
  Matrix26 J_analytic = dLineMeasurement_dPose(T_B_W, T_C_B, n, lines[0].anchorPoint(), lines[0].direction());

  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(J_numeric, J_analytic));
}

ZE_UNITTEST_ENTRYPOINT
