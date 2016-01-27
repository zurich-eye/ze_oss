#include <iostream>
#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>
#include <ze/cameras/camera_rig.h>

TEST(CameraRigTests, testYamlLoading)
{
  std::string data_dir = ze::getTestDataDir("camera_models");
  std::string yaml_file = data_dir + "/camera_rig_1.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));
  ze::CameraRig::Ptr rig = ze::CameraRig::loadFromYaml(yaml_file);
  rig->print(std::cout);
}

ZE_UNITTEST_ENTRYPOINT
