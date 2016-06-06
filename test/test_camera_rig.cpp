#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>
#include <ze/cameras/camera.h>
#include <ze/cameras/camera_rig.h>

TEST(CameraRigTests, testYamlLoading)
{
  using namespace ze;
  CameraRig::Ptr rig =
      CameraRig::loadFromYaml(joinPath(getTestDataDir("camera_models"),
                                       "camera_rig_1.yaml"));

  EXPECT_EQ(rig->size(), 2);
  EXPECT_NEAR(rig->at(0).projectionParameters()(0), 458.654, 1e-3);
  EXPECT_NEAR(rig->at(1).projectionParameters()(0), 457.587, 1e-3);
  EXPECT_NEAR(rig->T_C_B(0).getTransformationMatrix()(0, 0), 0.0148655, 1e-3);
  EXPECT_NEAR(rig->T_C_B(1).getTransformationMatrix()(0, 0), 0.0125553, 1e-3);
  EXPECT_STREQ(rig->label().c_str(), "Euroc");
  EXPECT_STREQ(rig->at(0).label().c_str(), "cam0");
  EXPECT_STREQ(rig->at(1).label().c_str(), "cam1");
}

TEST(CameraRigTests, testStereoPairIdentification)
{
  using namespace ze;

  {
    CameraRig::Ptr rig =
        CameraRig::loadFromYaml(joinPath(getTestDataDir("camera_models"),
                                         "camera_rig_1.yaml"));
    StereoIndexPairs pairs = identifyStereoPairsInRig(*rig, 0.7, 0.1);
    EXPECT_EQ(pairs.size(), 1u);
  }

  {
    CameraRig::Ptr rig =
        CameraRig::loadFromYaml(joinPath(getTestDataDir("camera_models"),
                                         "camera_rig_2.yaml"));
    StereoIndexPairs pairs = identifyStereoPairsInRig(*rig, 0.7, 0.1);
    EXPECT_EQ(pairs.size(), 1u);
  }

}

ZE_UNITTEST_ENTRYPOINT
