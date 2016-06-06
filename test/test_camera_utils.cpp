#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_rig.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>

TEST(CameraUtilsTest, randomKeypoints)
{
  const int kMargin = 20;
  ze::Keypoints kps = ze::generateRandomKeypoints(ze::Size2u(640, 480), kMargin, 10);
  ASSERT_EQ(kps.cols(), 10);
  for(size_t i = 0; i < 10; ++i)
  {
    EXPECT_GE(kps(0,i), kMargin);
    EXPECT_GE(kps(1,i), kMargin);
    EXPECT_LT(kps(0,i), 640 - kMargin - 1);
    EXPECT_LT(kps(1,i), 480 - kMargin - 1);
  }
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
  ze::CameraRig::Ptr rig = ze::CameraRig::loadFromYaml(yaml_file);

  FloatType overlap1 = overlappingFieldOfView(*rig, 0u, 1u);
  FloatType overlap2 = overlappingFieldOfView(*rig, 1u, 0u);

  EXPECT_NEAR(overlap1, overlap2, 0.1);
}

ZE_UNITTEST_ENTRYPOINT

