#include <ze/imu/imu_rig.h>
#include <ze/imu/imu_yaml_serialization.h>

namespace ze {

ImuRig::ImuRig(
    const TransformationVector& T_C_B,
    const ImuVector& imus,
    const std::string& label)
  : T_C_B_(T_C_B)
  , imus_(imus)
  , label_(label)
{
  CHECK_EQ(T_C_B_.size(), imus_.size());
  for(size_t i = 0; i < size(); ++i)
  {
    CHECK_NOTNULL(imus_[i].get());
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
