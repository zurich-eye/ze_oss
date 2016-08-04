#include <functional>

#include <ze/cameras/camera_impl.h>
#include <ze/common/numerical_derivative.h>
#include <ze/common/test_entrypoint.h>
#include <ze/geometry/line.hpp>

TEST(LineTests, testLineGeneration)
{
  using namespace ze;
  // Create lines.
  size_t n = 100;
  Lines lines;
  Positions start, end;
  PinholeCamera cam = createTestPinholeCamera();
  Transformation T_W_C;
  T_W_C.setRandom();
  std::tie(start, end) = generateRandomVisibleLines(cam, T_W_C, n, lines);

  for (size_t i = 0; i < n; ++i)
  {
    Vector3 direction = lines[i].direction();
    Position anchor_point = lines[i].anchorPoint();
    // Check whether anchor point is really the closest point available.
    EXPECT_NEAR(direction.dot(anchor_point), 0.0, 1e-10);
    // Direction should be a normalized vector.
    EXPECT_NEAR(direction.norm(), 1.0, 1e-15);
    // Anchor, start and end point should be on the line.
    EXPECT_NEAR(lines[i].distanceToLine(anchor_point), 0.0, 1e-14);
    EXPECT_NEAR(lines[i].distanceToLine(start.col(i)), 0.0, 1e-13);
    EXPECT_NEAR(lines[i].distanceToLine(end.col(i)), 0.0, 1e-14);
  }
}

TEST(LineTests, testJacobian)
{
  using namespace ze;
  Transformation T_B_W, T_C_B;
  T_B_W.setRandom();
  T_C_B.setRandom();
  const Transformation T_C_W = T_C_B * T_B_W;
  const Transformation T_W_C = T_C_W.inverse();

  size_t num_tests = 100;
  Lines lines_W;
  Positions start_W, end_W;
  ze::PinholeCamera cam = createTestPinholeCamera();
  std::tie(start_W, end_W) = generateRandomVisibleLines(cam, T_W_C, num_tests, lines_W);

  // Create measurements.
  LineMeasurements measurements_C(3, num_tests);
  size_t i;
  for (i = 0; i < num_tests; ++i)
  {
    measurements_C.col(i) =
        lineMeasurementFromBearings(T_C_W * start_W.col(i),
                                    T_C_W * end_W.col(i));
  }

  auto measurementError = [&](const Transformation& T_B_W_in_lambda) {
    Transformation T_W_C_in_lambda = (T_C_B * T_B_W_in_lambda).inverse();
    Position camera_pos_W = T_W_C_in_lambda.getPosition();
    Vector3 measurement_W = T_W_C_in_lambda.getRotation().rotate(measurements_C.col(i));
    return lines_W[i].calculateMeasurementError(measurement_W, camera_pos_W);
  };

  LineMeasurements measurements_W = T_W_C.getRotationMatrix() * measurements_C;
  for (i = 0; i < num_tests; ++i)
  {
    Matrix26 J_numeric =
        numericalDerivative<Vector2, Transformation>(measurementError, T_B_W);
    Matrix26 J_analytic =
        dLineMeasurement_dPose(T_B_W, T_C_B, measurements_W.col(i),
                               lines_W[i].anchorPoint(), lines_W[i].direction());
    EXPECT_TRUE(EIGEN_MATRIX_NEAR(J_numeric, J_analytic, 1e-9));
  }

}

ZE_UNITTEST_ENTRYPOINT
