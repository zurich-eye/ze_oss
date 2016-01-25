#include <string>
#include <vector>
#include <iostream>
#include <functional>

#include <ze/common/test/entrypoint.h>
#include <ze/common/test/utils.h>
#include <ze/common/matrix.h>
#include <ze/common/manifold.h>
#include <ze/common/numerical_derivative.h>
#include <ze/cameras/camera_impl.h>
#include <ze/cameras/camera_utils.h>

TEST(CameraImplTests, testPinholeJacobian)
{
  using namespace ze;
  PinholeCamera cam = createPinholeCamera(752, 480, 310, 320, 376.0, 240.0);
  Vector3 bearing = cam.backProject(Vector2(200, 300));
  Vector2 px = cam.project(bearing);
  ASSERT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(px, Vector2(200, 300)));
  Matrix23 H = cam.dProject_dLandmark(bearing);
  Matrix23 H_numerical =
      numericalDerivative<Vector2, Vector3>(
        std::bind(&PinholeCamera::project, &cam, std::placeholders::_1), bearing);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
}

TEST(CameraImplTests, testRadTanJacobian)
{
  using namespace ze;
  RadTanCamera cam = createRadTanCamera(752, 480, 310, 320, 376.0, 240.0,
                                        -0.2834, 0.0739, 0.00019, 1.76e-05);
  Vector3 bearing = cam.backProject(Vector2(200, 300));
  Vector2 px = cam.project(bearing);
  ASSERT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200, 300), 1e-2));
  Matrix23 H = cam.dProject_dLandmark(bearing);
  Matrix23 H_numerical =
      numericalDerivative<Vector2, Vector3>(
        std::bind(&RadTanCamera::project, &cam, std::placeholders::_1), bearing);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
}

TEST(CameraImplTests, testYamlParsing)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_nodistortion.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);
  cam->print(std::cout);
  EXPECT_DOUBLE_EQ(cam->params()(0), 320.0);
  EXPECT_DOUBLE_EQ(cam->params()(1), 310.0);
  EXPECT_DOUBLE_EQ(cam->params()(2), 376.5);
  EXPECT_DOUBLE_EQ(cam->params()(3), 240.5);
}

TEST(CameraImplTests, testVectorized)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_nodistortion.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);

  constexpr size_t N = 100;
  ze::Keypoints kps1 = ze::generateRandomKeypoints(cam->width(), cam->height(), 15, N);
  ze::Bearings bearings = cam->backProjectVectorized(kps1);
  ze::normalizeBearings(bearings);
  ze::Keypoints kps2 = cam->projectVectorized(bearings);
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(kps1, kps2));
  EXPECT_DOUBLE_EQ(bearings.col(0).norm(), 1.0);

  Eigen::Matrix<double, 6, N> H_vec = cam->dProject_dLandmarkVectorized(bearings);
  Eigen::Matrix<double, 2, 3> H1 = cam->dProject_dLandmark(bearings.col(0));
  Eigen::Matrix<double, 2, 3> H2 = Eigen::Map<Eigen::Matrix<double, 2, 3>>(H_vec.col(0).data());
  EXPECT_TRUE(EIGEN_MATRIX_EQUAL_DOUBLE(H1, H2));
}

ZE_UNITTEST_ENTRYPOINT
