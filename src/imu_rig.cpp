// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/imu/imu_rig.h>
#include <ze/imu/imu_yaml_serialization.h>

namespace ze {

ImuRig::ImuRig(
    const TransformationVector& T_B_S,
    const ImuVector& imus,
    const std::string& label)
  : T_B_S_(T_B_S)
  , imus_(imus)
  , label_(label)
{
  CHECK_EQ(T_B_S_.size(), imus_.size());
  for(size_t i = 0; i < size(); ++i)
  {
    CHECK_NOTNULL(imus_[i].get());
  }

  // set inverse transformations
  for(size_t i = 0; i < size(); ++i)
  {
    T_S_B_.push_back(T_B_S_[i].inverse());
  }
}

ImuRig::Ptr ImuRig::loadFromYaml(const std::string& yaml_file)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
    return doc.as<ImuRig::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Cannot load ImuRig from file:" << yaml_file << "\n"
               << ex.what();
  }
  return nullptr;
}
} // namespace ze
