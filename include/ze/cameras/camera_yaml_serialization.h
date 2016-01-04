#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace ze {
class Camera;
}  // namespace ze

namespace YAML {

template<>
struct convert<std::shared_ptr<ze::Camera>>
{
  // Parse camera from yaml node.
  static bool decode(const Node& node, std::shared_ptr<ze::Camera>& camera);
  static Node encode(const std::shared_ptr<ze::Camera>& camera);
};

template<>
struct convert<ze::Camera>
{
  static bool decode(const Node& node, ze::Camera& camera);
  static Node encode(const ze::Camera& camera);
};

}  // namespace YAML
