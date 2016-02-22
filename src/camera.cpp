#include <ze/cameras/camera.h>

#include <string>
#include <ze/cameras/camera_yaml_serialization.h>

namespace ze {

Camera::Camera(const int width, const int height, const CameraType type,
               const VectorX& projection_params, const VectorX& distortion_params)
  : width_(width)
  , height_(height)
  , projection_params_(projection_params)
  , distortion_params_(distortion_params)
  , type_(type)
{}

Bearings Camera::backProjectVectorized(const Keypoints& px_vec) const
{
  Bearings bearings(3, px_vec.cols());
  for(int i = 0; i < px_vec.cols(); ++i)
  {
    bearings.col(i) = this->backProject(px_vec.col(i));
  }
  return bearings;
}

Keypoints Camera::projectVectorized(const Bearings& bearing_vec) const
{
  Keypoints px_vec(2, bearing_vec.cols());
  for(int i = 0; i < bearing_vec.cols(); ++i)
  {
    px_vec.col(i) = this->project(bearing_vec.col(i));
  }
  return px_vec;
}

Matrix6X Camera::dProject_dLandmarkVectorized(const Positions& pos_vec) const
{
  Matrix6X J_vec(6, pos_vec.cols());
  for(int i = 0; i < pos_vec.cols(); ++i)
  {
    J_vec.col(i) =
        Eigen::Map<Matrix61>(this->dProject_dLandmark(pos_vec.col(i)).data());
  }
  return J_vec;
}

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
      << "  Model = " << typeString() << "\n"
      << "  Dimensions = " << width_ << "x" << height_ << "\n"
      << "  Proj. parameters = " << projection_params_.transpose() << "\n"
      << "  Dist. parameters = " << distortion_params_.transpose() << std::endl;
}

std::string Camera::typeString() const
{
  switch (type_)
  {
    case CameraType::Pinhole: return "Pinhole";
    case CameraType::PinholeFov: return "PinholeFov";
    case CameraType::PinholeEquidistant: return "PinholeEquidistant";
    case CameraType::PinholeRadialTangential: return "PinholeRadialTangential";
    default:
      LOG(FATAL) << "Unknown parameter type";
  }
  return "";
}

} // namespace ze
