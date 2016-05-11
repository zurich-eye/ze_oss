#include <ze/cameras/camera_yaml_serialization.h>
#include <ze/cameras/camera_impl.h>
#include <ze/cameras/camera_rig.h>
#include <ze/common/yaml_serialization.h>
#include <ze/common/types.h>

namespace YAML {

//------------------------------------------------------------------------------
// Camera loading.
bool convert<std::shared_ptr<ze::Camera>>::decode(
    const Node& node, ze::Camera::Ptr& camera)
{
  camera.reset();
  try {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the camera because the node is not a map.";
      return false;
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
        camera = std::make_shared<ze::PinholeCamera>(
              width, height, ze::CameraType::Pinhole, intrinsics,
              distortion_parameters);
      }
      else if(camera_type == "pinhole" && distortion_type == "radial-tangential")
      {
        camera = std::make_shared<ze::RadTanCamera>(
              width, height, ze::CameraType::PinholeRadialTangential, intrinsics,
              distortion_parameters);
      }
      else if(camera_type == "pinhole" && distortion_type == "equidistant")
      {
        camera = std::make_shared<ze::EquidistantCamera>(
              width, height, ze::CameraType::PinholeEquidistant, intrinsics,
              distortion_parameters);
      }
      else if(camera_type == "pinhole" && distortion_type == "fisheye")
      {
        camera = std::make_shared<ze::FovCamera>(
              width, height, ze::CameraType::PinholeFov, intrinsics,
              distortion_parameters);
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
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// Camera writing.
Node convert<ze::Camera::Ptr>::encode(const ze::Camera::Ptr& camera)
{
  LOG(FATAL) << "Not implemented!";
  Node camera_node;
  return camera_node;
}

//------------------------------------------------------------------------------
// CameraRig loading.
bool convert<std::shared_ptr<ze::CameraRig>>::decode(
    const Node& node, ze::CameraRig::Ptr& camera_rig)
{
  camera_rig.reset();
  try {
    if (!node.IsMap())
    {
      LOG(ERROR) << "Parsing CameraRig failed because node is not a map.";
      return false;
    }

    std::string label = "";
    if (!YAML::safeGet<std::string>(node, "label", &label))
    {
      LOG(ERROR) << "Parsing CameraRig label failed.";
      return false;
    }

    const Node& cameras_node = node["cameras"];
    if (!cameras_node.IsSequence())
    {
      LOG(ERROR) << "Parsing CameraRig failed because 'cameras' is not a sequence.";
      return false;
    }

    size_t num_cameras = cameras_node.size();
    if (num_cameras == 0)
    {
      LOG(ERROR) << "Parsing CameraRig failed. Number of cameras is 0.";
      return false;
    }

    ze::TransformationVector T_Ci_B;
    ze::CameraVector cameras;
    for (size_t i = 0; i < num_cameras; ++i)
    {
      const Node& camera_node = cameras_node[i];
      if (!camera_node)
      {
        LOG(ERROR) << "Unable to get camera node for camera " << i;
        return false;
      }
      if (!camera_node.IsMap())
      {
        LOG(ERROR) << "Camera node for camera " << i << " is not a map.";
        return false;
      }

      ze::Camera::Ptr camera;
      if (!YAML::safeGet(camera_node, "camera", &camera))
      {
        LOG(ERROR) << "Unable to retrieve camera " << i;
        return false;
      }

      Eigen::Matrix4d T_B_C;
      if (!YAML::safeGet(camera_node, "T_B_C", &T_B_C))
      {
        LOG(ERROR) << "Unable to get extrinsic transformation T_B_C for camera " << i;
        return false;
      }

      cameras.push_back(camera);
      T_Ci_B.push_back(ze::Transformation(T_B_C).inverse());
    }

    camera_rig.reset(new ze::CameraRig(T_Ci_B, cameras, label));
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "YAML exception during parsing: " << ex.what();
    camera_rig.reset();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// CameraRig writing.
Node convert<std::shared_ptr<ze::CameraRig> >::encode(
    const std::shared_ptr<ze::CameraRig>& /*camera_rig*/)
{
  LOG(FATAL) << "Not implemented!";
  Node camera_rig_node;
  return camera_rig_node;
}

}  // namespace YAML

