#include <string>

#include <ze/common/test_entrypoint.h>
#include <ze/common/test_utils.h>
#include <ze/common/path_utils.h>

#include <ze/imu/imu_model.h>
#include <ze/imu/imu_rig.h>
#include <ze/imu/imu_yaml_serialization.h>

TEST(ImuYamlSerialization, testYamlLoading)
{
  std::string data_dir = ze::getTestDataDir("imu_models");
  std::string yaml_file = data_dir + "/zurich_eye_one.yaml";
  ASSERT_TRUE(ze::fileExists(yaml_file));

  ze::ImuRig::Ptr rig = ze::ImuRig::loadFromYaml(yaml_file);

  EXPECT_EQ(rig->size(), 2);
  EXPECT_STREQ(rig->label().c_str(), "zuricheyeonetest");
  EXPECT_STREQ(rig->at(0).label().c_str(), "bmx0");
  EXPECT_STREQ(rig->at(1).label().c_str(), "bmx1");
}

ZE_UNITTEST_ENTRYPOINT
