// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

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
  std::string yaml_file = ze::joinPath(data_dir, "/zurich_eye_one.yaml");
  ASSERT_TRUE(ze::fileExists(yaml_file));

  ze::ImuRig::Ptr rig = ze::ImuRig::loadFromYaml(yaml_file);

  EXPECT_EQ(rig->size(), 3);
  EXPECT_STREQ(rig->label().c_str(), "zuricheyeonetest");
  EXPECT_STREQ(rig->at(0).label().c_str(), "bmx0");
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignment,
            rig->at(0).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignment,
            rig->at(0).gyroscopeModel()->intrinsicModel()->type());

  EXPECT_STREQ(rig->at(1).label().c_str(), "bmx1");
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignmentSizeEffect,
            rig->at(1).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::ScaleMisalignmentGSensitivity,
            rig->at(1).gyroscopeModel()->intrinsicModel()->type());

  EXPECT_STREQ(rig->at(2).label().c_str(), "bmx2");
  EXPECT_EQ(ze::ImuIntrinsicType::Calibrated,
            rig->at(2).accelerometerModel()->intrinsicModel()->type());
  EXPECT_EQ(ze::ImuIntrinsicType::Calibrated,
            rig->at(2).gyroscopeModel()->intrinsicModel()->type());
}

ZE_UNITTEST_ENTRYPOINT
