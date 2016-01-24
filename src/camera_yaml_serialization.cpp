#include <ze/cameras/camera_yaml_serialization.h>
#include <ze/cameras/camera_impl.h>
#include <ze/common/yaml_serialization.h>
#include <ze/common/types.h>

namespace YAML {

bool convert<std::shared_ptr<ze::Camera>>::decode(const Node& node, ze::Camera::Ptr& camera)
{
  camera.reset();
  try {
    if(!node.IsMap()) {
      LOG(ERROR) << "Unable to get parse the camera because the node is not a map.";
      return true;
    }

    std::string camera_type;
    int width, height;
    Eigen::VectorXd intrinsics;
    const YAML::Node distortion_config = node["distortion"];
    std::string distortion_type;
    Eigen::VectorXd distortion_parameters;

    if(!distortion_config)
    {
      distortion_type = "none";
    }

    if(YAML::safeGet(distortion_config, "type", &distortion_type) &&
       YAML::safeGet(distortion_config, "parameters", &distortion_parameters) &&
       YAML::safeGet(node, "type", &camera_type) &&
       YAML::safeGet(node, "image_width", &width) &&
       YAML::safeGet(node, "image_height", &height) &&
       YAML::safeGet(node, "intrinsics", &intrinsics))
    {
      if(camera_type == "pinhole" && distortion_type == "none")
      {
        VLOG(1) << "load pinhole camera without distortion";
        camera = std::make_shared<ze::PinholeCamera>(
              width, height, ze::CameraType::Pinhole, intrinsics, ze::Vector());
      }
      else if(camera_type == "pinhole" && distortion_type == "radial-tangential")
      {
        LOG(FATAL) << "Camera model not yet supported.";
      }
      else if(camera_type == "pinhole" && distortion_type == "equidistant")
      {
        LOG(FATAL) << "Camera model not yet supported.";
      }
      else if(camera_type == "pinhole" && distortion_type == "fisheye")
      {
        LOG(FATAL) << "Camera model not yet supported.";
      }
      else
      {
        LOG(FATAL) << "Camera model not yet supported.";
      }
    }

    if(node["label"])
      camera->setLabel(node["label"].as<std::string>());
  }
  catch(const std::exception& e)
  {
    LOG(ERROR) << "YAML exception during parsing: " << e.what();
    camera.reset();
    return true;
  }
  return true;
}

Node convert<ze::Camera::Ptr>::encode(const ze::Camera::Ptr& camera)
{
  return convert<ze::Camera>::encode(*CHECK_NOTNULL(camera.get()));
}

bool convert<ze::Camera>::decode(const Node& /*node*/, ze::Camera& /*camera*/)
{
  LOG(FATAL) << "Not implemented!";
  return false;
}

Node convert<ze::Camera>::encode(const ze::Camera& /*camera*/)
{
  LOG(FATAL) << "Not implemented!";
  Node camera_node;
  return camera_node;
}

}  // namespace YAML

