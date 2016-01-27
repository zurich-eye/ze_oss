#pragma once

#include <memory>

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace ze {
class Camera;
class CameraRig;
}  // namespace ze

namespace YAML {

template<>
struct convert<std::shared_ptr<ze::Camera>>
{
  static bool decode(const Node& node, std::shared_ptr<ze::Camera>& camera);
  static Node encode(const std::shared_ptr<ze::Camera>& camera);
};

template<>
struct convert<std::shared_ptr<ze::CameraRig>>
{
  static bool decode(const Node& node, std::shared_ptr<ze::CameraRig>& camera);
  static Node encode(const std::shared_ptr<ze::CameraRig>& camera_rig);
};

}  // namespace YAML
