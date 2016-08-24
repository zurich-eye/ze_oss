// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <memory>

#include <ze/common/logging.hpp>
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
// The yaml-cpp version in yaml_cpp_catkin uses auto_ptr which is deprecated.
#include <yaml-cpp/yaml.h>
#pragma diagnostic pop

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
