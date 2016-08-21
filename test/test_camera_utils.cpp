#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_rig.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>
#include <ze/common/config.hpp>
#ifdef ZE_USE_OPENCV
#include <opencv2/highgui/highgui.hpp>
#endif

TEST(CameraUtilsTest, randomKeypoints)
{
  const int margin = 20;
  const int num_keypoints = 200;
  ze::Keypoints kps = ze::generateRandomKeypoints(ze::Size2u(640, 480), margin, num_keypoints);
  ASSERT_EQ(kps.cols(), num_keypoints);
  for (size_t i = 0; i < num_keypoints; ++i)
  {
    EXPECT_GE(kps(0,i), margin);
    EXPECT_GE(kps(1,i), margin);
    EXPECT_LT(kps(0,i), 640 - margin - 1);
    EXPECT_LT(kps(1,i), 480 - margin - 1);
  }

#ifdef ZE_USE_OPENCV
  if (false)
  {
    cv::Mat img(480, 640, CV_8UC1, cv::Scalar(0));
    for (size_t i = 0; i < num_keypoints; ++i)
    {
      cv::circle(img, cv::Point(kps(0,i), kps(1,i)), 2, cv::Scalar(255));
    }
    cv::imshow("img", img);
    cv::waitKey(0);
  }
#endif
}

TEST(CameraUtilsTest, uniformKeypoints)
{
  using namespace ze;

  {
    uint32_t margin = 0;
    Size2u img_size(752, 480);
    Keypoints kps = generateUniformKeypoints(img_size, margin, 50);
    for (int i = 0; i < kps.cols(); ++i)
    {
      EXPECT_TRUE(isVisibleWithMargin(img_size, kps.col(i), margin));
    }
  }

  {
    uint32_t margin = 10;
    Size2u img_size(752, 480);
    Keypoints kps = generateUniformKeypoints(img_size, margin, 50);
    for (int i = 0; i < kps.cols(); ++i)
    {
      EXPECT_TRUE(isVisibleWithMargin(img_size, kps.col(i), margin));
    }
  }
}

TEST(CameraUtilsTest, overlappingFieldOfView)
{
  using namespace ze;

  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = ze::joinPath(data_dir, "camera_rig_1.yaml");
  ze::CameraRig::Ptr rig = ze::cameraRigFromYaml(yaml_file);

  real_t overlap1 = overlappingFieldOfView(*rig, 0u, 1u);
  real_t overlap2 = overlappingFieldOfView(*rig, 1u, 0u);

  EXPECT_NEAR(overlap1, overlap2, 0.1);
}

ZE_UNITTEST_ENTRYPOINT

