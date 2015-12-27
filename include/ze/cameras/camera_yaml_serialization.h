#pragma once

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>

namespace ze {
template<typename Scalar>
class CameraInterface;
}  // namespace ze

namespace YAML {

template<>
struct convert<std::shared_ptr<ze::CameraInterface<double>>> {
  /// This function will attempt to parse a camera from the yaml node.
  /// By default, yaml-cpp will throw and exception if the parsing fails.
  /// This function was written to *not* throw exceptions. Hence, decode always
  /// returns true, but when it fails, the shared pointer will be null.
  static bool decode(const Node& node, std::shared_ptr<ze::CameraInterface<double>>& camera);
  static Node encode(const std::shared_ptr<ze::CameraInterface<double>>& camera);
};

template<>
struct convert<ze::CameraInterface<double>> {
  static bool decode(const Node& node, ze::CameraInterface<double>& camera);
  static Node encode(const ze::CameraInterface<double>& camera);
};

}  // namespace YAML
