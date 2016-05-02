#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>
#include <ze/cameras/camera.h>
#include <ze/cameras/camera_rig.h>

TEST(CameraRigTests, testYamlLoading)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_rig_1.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::CameraRig::Ptr rig = ze::CameraRig::loadFromYaml(yaml_file);

  EXPECT_EQ(rig->size(), 2);
  EXPECT_NEAR(rig->at(0).projectionParameters()(0), 458.654, 1e-3);
  EXPECT_NEAR(rig->at(1).projectionParameters()(0), 457.587, 1e-3);
  EXPECT_NEAR(rig->T_C_B(0).getTransformationMatrix()(0, 0), 0.0148655, 1e-3);
  EXPECT_NEAR(rig->T_C_B(1).getTransformationMatrix()(0, 0), 0.0125553, 1e-3);
  EXPECT_STREQ(rig->label().c_str(), "Euroc");
  EXPECT_STREQ(rig->at(0).label().c_str(), "cam0");
  EXPECT_STREQ(rig->at(1).label().c_str(), "cam1");
}

ZE_UNITTEST_ENTRYPOINT
