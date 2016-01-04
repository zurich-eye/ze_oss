#include <string>
#include <vector>
#include <iostream>
#include <functional>

#include <ze/common/test/entrypoint.h>
#include <ze/common/manifold.h>
#include <ze/common/numerical_derivative.h>
#include <ze/cameras/camera_impl.h>

TEST(CameraPinholeTest, testProjectionJacobian)
{
  ze::PinholeCamera cam(752, 480, 310, 320, 376.0, 240.0);
  cam.setLabel("test");
  cam.print(std::cout);

  Eigen::Vector3d bearing = cam.backProject(Eigen::Vector2d(200, 300));
  Eigen::Vector2d px = cam.project(bearing);
  CHECK(EIGEN_MATRIX_EQUAL_DOUBLE(px, Eigen::Vector2d(200, 300)));
  Eigen::Matrix<double, 2, 3> H = cam.dProject_dBearing(bearing);
  Eigen::Matrix<double, 2, 3> H_numerical =
      ze::numericalDerivative<Eigen::Vector2d, Eigen::Vector3d>(
        std::bind(&ze::PinholeCamera::project, &cam, std::placeholders::_1),
        bearing);
  CHECK(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
}

ZE_UNITTEST_ENTRYPOINT
