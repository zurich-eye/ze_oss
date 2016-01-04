#include <ze/cameras/camera.h>

#include <string>
#include <ze/cameras/camera_yaml_serialization.h>

namespace ze {

Camera::Camera(const int width, const int height, const CameraType type,
               const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& params)
  : width_(width)
  , height_(height)
  , params_(params)
  , type_(type)
{}

Camera::Ptr Camera::loadFromYaml(const std::string& path)
{
  try
  {
    YAML::Node doc = YAML::LoadFile(path.c_str());
    return doc.as<Camera::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Failed to load Camera from file " << path << " with the error: \n"
               << ex.what();
  }
  return Camera::Ptr();
}

void Camera::print(std::ostream& out, const std::string& s) const
{
  out << s << std::endl
      << "  Label = " << label_ << "\n"
      << "  Model = " << cameraTypeString(type_) << "\n"
      << "  Dimensions = " << width_ << "x" << height_ << "\n"
      << "  Parameters = " << params_.transpose() << std::endl;
}

std::string cameraTypeString(CameraType type)
{
  switch (type)
  {
    case CameraType::kPinhole: return "Pinhole";
    case CameraType::kPinholeFov: return "PinholeFov";
    case CameraType::kPinholeEquidistant: return "PinholeEquidistant";
    case CameraType::kPinholeRadialTangential: return "PinholeRadialTangential";
    default:
      LOG(FATAL) << "Unknown parameter type";
  }
  return "";
}

} // namespace ze
