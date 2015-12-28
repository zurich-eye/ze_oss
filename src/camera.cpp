#include <string>
#include <ze/cameras/camera.h>

namespace ze {

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
