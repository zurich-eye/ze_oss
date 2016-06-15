#include <ze/cameras/camera_rig.h>
#include <ze/cameras/camera_yaml_serialization.h>

namespace ze {

CameraRig::CameraRig(
    const TransformationVector& T_C_B,
    const CameraVector& cameras,
    const std::string& label)
  : T_C_B_(T_C_B)
  , cameras_(cameras)
  , label_(label)
{
  CHECK_EQ(T_C_B_.size(), cameras_.size());
  for(size_t i = 0; i < size(); ++i)
  {
    CHECK_NOTNULL(cameras_[i].get());
  }
}

CameraRig::Ptr CameraRig::loadFromYaml(const std::string& yaml_file)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
    return doc.as<CameraRig::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Cannot load CameraRig from file:" << yaml_file << "\n"
               << ex.what();
  }
  return nullptr;
}

CameraRig::Ptr CameraRig::getSubRig(
    const std::vector<uint32_t>& camera_indices,
    const std::string& label)
{
  CameraVector cameras;
  TransformationVector T;
  for (uint32_t i : camera_indices)
  {
    cameras.push_back(atShared(i));
    T.push_back(T_C_B(i));
  }
  return std::make_shared<CameraRig>(T, cameras, label);
}

std::ostream& operator<<(std::ostream& out, const CameraRig& rig)
{
  out << "Camera Rig: \n"
      << "  Label = " << rig.label() << "\n";
  for (size_t i = 0; i < rig.size(); ++i)
  {
    out << "- Camera " << i << "\n"
        << rig.at(i) << "\n"
        << "    T_B_C = \n" << rig.T_C_B(i).inverse() << "\n";
  }
  return out;
}

} // namespace ze
