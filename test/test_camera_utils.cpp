#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/cameras/camera_utils.h>

TEST(CameraUtilsTest, randomKeypoints)
{
  const int kMargin = 20;
  ze::Keypoints kps = ze::generateRandomKeypoints(640, 480, kMargin, 10);
  ASSERT_EQ(kps.cols(), 10);
  for(size_t i = 0; i < 10; ++i)
  {
    EXPECT_GE(kps(0,i), kMargin);
    EXPECT_GE(kps(1,i), kMargin);
    EXPECT_LT(kps(0,i), 640 - kMargin - 1);
    EXPECT_LT(kps(1,i), 480 - kMargin - 1);
  }
}




ZE_UNITTEST_ENTRYPOINT
