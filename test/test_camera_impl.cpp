#include <string>
#include <vector>
#include <iostream>
#include <functional>

#include <ze/common/benchmark.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/matrix.h>
#include <ze/common/manifold.h>
#include <ze/common/numerical_derivative.h>
#include <ze/cameras/camera_rig.h>
#include <ze/cameras/camera.h>
#include <ze/cameras/camera_impl.h>
#include <ze/cameras/camera_utils.h>

namespace ze {

void cameraTest(const Camera& cam, const std::string& test_name)
{
  uint32_t N = 300;
  Keypoints px1 = generateRandomKeypoints(cam.width(), cam.height(), 10u, N);
  Bearings f1 = cam.backProjectVectorized(px1);
  Keypoints px2 = cam.projectVectorized(f1);
  Keypoints px_error = px1 - px2;
  FloatType max_error = px_error.colwise().norm().array().maxCoeff();
  EXPECT_LT(max_error, 1e-4);

  // Test back-project vectorized
  auto backProjectVectorizedLambda = [&](){
    f1 = cam.backProjectVectorized(px1);
  };
  runTimingBenchmark(backProjectVectorizedLambda, 10, 20,
                     test_name + ": Back-project vectorized", true);

  // Test not vectorized.
  auto backProjectLambda = [&](){
    for (size_t i = 0; i < N; ++i)
    {
      f1.col(i) = cam.backProjectVectorized(px1.col(i));
    }
  };
  runTimingBenchmark(backProjectLambda, 10, 20,
                     test_name + ": Back-project N-times", true);

  // Test back-project vectorized
  auto projectVectorizedLambda = [&](){
    px2 = cam.projectVectorized(f1);
  };
  runTimingBenchmark(projectVectorizedLambda, 10, 20,
                     test_name + ": Project vectorized", true);

  // Test not vectorized.
  auto projectLambda = [&](){
    for (size_t i = 0; i < N; ++i)
    {
      px2.col(i) = cam.project(f1.col(i));
    }
  };
  runTimingBenchmark(projectLambda, 10, 20,
                     test_name + ": Project N-times", true);
  VLOG(1) << "================================";
}

} // namespace ze

TEST(CameraImplTests, testPinhole)
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
  cameraTest(cam, "Pinhole");
}

TEST(CameraImplTests, testFov)
{
  using namespace ze;
  FovCamera cam = createFovCamera(752, 480, 310, 320, 376.0, 240.0, 0.947367);
  Vector3 bearing = cam.backProject(Vector2(200, 300));
  Vector2 px = cam.project(bearing);
  ASSERT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200, 300), 1e-4));
  Matrix23 H = cam.dProject_dLandmark(bearing);
  Matrix23 H_numerical =
      numericalDerivative<Vector2, Vector3>(
        std::bind(&FovCamera::project, &cam, std::placeholders::_1), bearing);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
  cameraTest(cam, "Fov");
}

TEST(CameraImplTests, testRadTan)
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
  cameraTest(cam, "RadTan");
}

TEST(CameraImplTests, testEquidistant)
{
  using namespace ze;
  EquidistantCamera cam = createEquidistantCamera(752, 480, 310, 320, 376.0, 240.0,
                                                  -0.00279, 0.02414, -0.04304, 0.03118);
  Vector3 bearing = cam.backProject(Vector2(200, 300));
  Vector2 px = cam.project(bearing);
  ASSERT_TRUE(EIGEN_MATRIX_NEAR(px, Vector2(200, 300), 1e-2));
  Matrix23 H = cam.dProject_dLandmark(bearing);
  Matrix23 H_numerical =
      numericalDerivative<Vector2, Vector3>(
        std::bind(&EquidistantCamera::project, &cam, std::placeholders::_1), bearing);
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(H, H_numerical, 1e-6));
  cameraTest(cam, "Equidistant");
}

TEST(CameraImplTests, testYamlParsingPinhole)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_nodistortion.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(1), 310.0);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(2), 376.5);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(3), 240.5);
}

TEST(CameraImplTests, testYamlParsingFoV)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_fov.yaml";
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_DOUBLE_EQ(cam->distortionParameters()(0), 0.940454);
}

TEST(CameraImplTests, testYamlParsingRadTan)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_radtan.yaml";
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_DOUBLE_EQ(cam->distortionParameters()(0), -0.28340811217029355);
}

TEST(CameraImplTests, testYamlParsingEquidistant)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_pinhole_equidistant.yaml";
  ze::Camera::Ptr cam = ze::Camera::loadFromYaml(yaml_file);
  EXPECT_DOUBLE_EQ(cam->projectionParameters()(0), 320.0);
  EXPECT_DOUBLE_EQ(cam->distortionParameters()(0), -0.0027973061697674074);
}

ZE_UNITTEST_ENTRYPOINT
