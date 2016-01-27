#include <ze/cameras/camera_rig.h>
#include <ze/cameras/camera.h>
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

void CameraRig::print(std::ostream& out, const std::string& s) const
{
  out << s << "\n";
  for(size_t i = 0; i < size(); ++i)
  {
    out << "Camera #" << i << ": " << label_ << "\n";
    cameras_[i]->print(out, "");
    out << "  T_C_B = " << T_C_B_[i] << std::endl;
  }
}

} // namespace ze
