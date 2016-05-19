#include <ze/imu/accelerometer_model.h>
#include <ze/imu/gyroscope_model.h>
#include <ze/imu/imu_intrinsic_model.h>
#include <ze/imu/imu_model.h>
#include <ze/imu/imu_noise_model.h>
#include <ze/imu/imu_rig.h>
#include <ze/imu/imu_yaml_serialization.h>
#include <ze/common/types.h>
#include <ze/common/yaml_serialization.h>

using ze::Matrix3;
using ze::Vector3;
using ze::FloatType;

namespace YAML {

//------------------------------------------------------------------------------
// IMU loading.
bool convert<std::shared_ptr<ze::ImuModel>>::decode(
    const Node& node, std::shared_ptr<ze::ImuModel>& imu)
{
  imu.reset();
  try {
    if(!node.IsMap())
    {
      LOG(ERROR) << "Unable to get parse the imu because the node is not a map.";
      return false;
    }

    const YAML::Node gyroscopes = node["gyroscopes"];
    const YAML::Node accelerometers = node["accelerometers"];

    if (!gyroscopes || !accelerometers)
    {
      throw std::runtime_error("Missing Gyroscopes / Accelerometer keys.");
    }

    // get the types
    const YAML::Node g_noise_node = gyroscopes["noise_model"];
    const YAML::Node g_intrinsic_node = gyroscopes["intrinsic_model"];
    const YAML::Node a_noise_node = accelerometers["noise_model"];
    const YAML::Node a_intrinsic_node = accelerometers["intrinsic_model"];

    std::string g_noise_type;
    std::string g_intrinsic_type;
    std::string a_noise_type;
    std::string a_intrinsic_type;
    if (!YAML::safeGet(g_noise_node, "type", &g_noise_type) ||
        !YAML::safeGet(g_intrinsic_node, "type", &g_intrinsic_type) ||
        !YAML::safeGet(a_noise_node, "type", &a_noise_type) ||
        !YAML::safeGet(a_intrinsic_node, "type", &a_intrinsic_type))
    {
      throw std::runtime_error("Missing Gyroscope or Accelerometer types.");
    }

    if (!internal::validateNoise(g_noise_type) ||
        !internal::validateNoise(a_noise_type) ||
        !internal::validateIntrinsic(g_intrinsic_type) ||
        !internal::validateIntrinsic(a_intrinsic_type))
    {
      throw std::runtime_error("Invalid Intrinsic or Noise type.");
    }

    ze::ImuNoiseModel::Ptr g_noise = internal::decodeNoise(g_noise_node);
    ze::ImuNoiseModel::Ptr a_noise = internal::decodeNoise(a_noise_node);

    ze::ImuIntrinsicModel::Ptr g_intrinsic_model =
        internal::decodeIntrinsics(g_intrinsic_node);
    ze::ImuIntrinsicModel::Ptr a_intrinsic_model =
        internal::decodeIntrinsics(a_intrinsic_node);

    ze::GyroscopeModel::Ptr gyro =
        std::make_shared<ze::GyroscopeModel>(g_intrinsic_model, g_noise);
    ze::AccelerometerModel::Ptr accel =
        std::make_shared<ze::AccelerometerModel>(a_intrinsic_model, a_noise);

    imu = std::make_shared<ze::ImuModel>(accel, gyro);

    if(node["label"])
    {
      imu->setLabel(node["label"].as<std::string>());
    }
    if(node["id"])
    {
      imu->setId(node["id"].as<std::string>());
    }
  }
  catch(const std::exception& e)
  {
    LOG(ERROR) << "YAML exception during parsing: " << e.what();
    imu.reset();
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
// Validation of types

bool internal::validateNoise(const std::string& value)
{
  if ("white-brownian" == value)
  {
    return true;
  }

  return false;
}

bool internal::validateIntrinsic(const std::string& value)
{
  if ("calibrated" == value ||
      "scale-misalignment" == value ||
      "scale-misalignment-gsensitivity" == value ||
      "scale-misalignment-size-effect" == value)
  {
    return true;
  }

  return false;
}

//------------------------------------------------------------------------------
// Noise Model loading.
std::shared_ptr<ze::ImuNoiseModel> internal::decodeNoise(const Node& node)
{
  if(!node.IsMap())
  {
    LOG(ERROR) << "Unable to get parse the imu because the node is not a map.";
    return false;
  }

  // gyroscopes:
  std::string noise_type;
  if (!YAML::safeGet(node, "type", &noise_type))
  {
    throw std::runtime_error("Missing Noise Model Type");
  }
  if (noise_type == "white-brownian")
  {
    FloatType noise_density;
    uint32_t bandwidth;
    FloatType bias_noise_density;
    if (!YAML::safeGet(node, "noise_density", &noise_density)
        || !YAML::safeGet(node, "bandwidth", &bandwidth)
        || !YAML::safeGet(node, "bias_noise_density", &bias_noise_density))
    {
      throw std::runtime_error("Missing Gyroscope Noise Parameters");
    }
    return std::make_shared<ze::ImuNoiseWhiteBrownian>(
          noise_density, bandwidth, bias_noise_density);
  }
  else
  {
    throw std::runtime_error("Unsupported Noise Model.");
  }
}

//------------------------------------------------------------------------------
// Intrinsic Model loading.
typename std::shared_ptr<ze::ImuIntrinsicModel> internal::decodeIntrinsics(
    const Node& node)
{
  if(!node.IsMap())
  {
    LOG(ERROR) << "Unable to get parse intrinsic model because the node is not a map.";
    return false;
  }

  // gyroscopes:
  std::string type;
  if (!YAML::safeGet(node, "type", &type))
  {
    throw std::runtime_error("Missing Intrinsic Model Type");
  }

  if (type == "calibrated")
  {
    return std::make_shared<ze::ImuIntrinsicModelCalibrated>();
  }
  else if (type == "scale-misalignment" ||
           type == "scale-misalignment-gsensitivity" ||
           type == "scale-misalignment-size-effect")
  {
    FloatType delay;
    uint32_t range;
    Vector3 b;
    Matrix3 M;

    if (!YAML::safeGet(node, "delay", &delay)
        || !YAML::safeGet(node, "range", &range)
        || !YAML::safeGet(node, "b", &b)
        || !YAML::safeGet(node, "M", &M))
    {
      throw std::runtime_error("Incomplete Intrinsic Parameters");
    }

    if ("scale-misalignment" == type)
    {
      return std::make_shared<ze::ImuIntrinsicModelScaleMisalignment>(
                         delay, range, b, M);
    }
    else if ("scale-misalignment-gsensitivity" == type)
    {
      Matrix3 Ma;
      if (!YAML::safeGet(node, "Ma", &Ma))
      {
        throw std::runtime_error("Incomplete Intrinsic Parameters");
      }
      return std::make_shared<
                       ze::ImuIntrinsicModelScaleMisalignmentGSensitivity>(
                         delay, range, b, M, Ma);
    }
    else if ("scale-misalignment-size-effect" == type)
    {
      Matrix3 R;
      if (!YAML::safeGet(node, "R", &R))
      {
        throw std::runtime_error("Incomplete Intrinsic Parameters");
      }
      return std::make_shared<
                       ze::ImuIntrinsicModelScaleMisalignmentSizeEffect>(
                         delay, range, b, M, R);
    }
  }
  else
  {
    throw std::runtime_error("Unsupported Intrinsic Model.");
  }

  return nullptr;
}

//------------------------------------------------------------------------------
// ImuRig loading.
bool convert<std::shared_ptr<ze::ImuRig>>::decode(
    const Node& node, std::shared_ptr<ze::ImuRig>& imu_rig)
{
  imu_rig.reset();
  try {
    if (!node.IsMap())
    {
      LOG(ERROR) << "Parsing ImuRig failed because node is not a map.";
      return false;
    }

    std::string label = "";
    if (!YAML::safeGet<std::string>(node, "label", &label))
    {
      LOG(ERROR) << "Parsing ImuRig label failed.";
      return false;
    }

    const Node& imus_node = node["imus"];
    if (!imus_node.IsSequence())
    {
      LOG(ERROR) << "Parsing ImuRig failed because 'imus' is not a sequence.";
      return false;
    }

    size_t num_imus = imus_node.size();
    if (num_imus == 0)
    {
      LOG(ERROR) << "Parsing ImuRig failed. Number of imus is 0.";
      return false;
    }

    ze::TransformationVector T_B_Si;
    ze::ImuVector imus;
    for (size_t i = 0; i < num_imus; ++i)
    {
      const Node& imu_node = imus_node[i];
      if (!imu_node)
      {
        LOG(ERROR) << "Unable to get imu node for imu " << i;
        return false;
      }
      if (!imu_node.IsMap())
      {
        LOG(ERROR) << "Imu node for imu " << i << " is not a map.";
        return false;
      }

      ze::ImuModel::Ptr imu;
      if (!YAML::safeGet(imu_node, "imu", &imu))
      {
        LOG(ERROR) << "Unable to retrieve imu " << i;
        return false;
      }

      Eigen::Matrix4d T_B_S;
      if (!YAML::safeGet(imu_node, "T_B_S", &T_B_S))
      {
        LOG(ERROR) << "Unable to get extrinsic transformation T_B_S for imu " << i;
        return false;
      }

      imus.push_back(imu);
      T_B_Si.push_back(ze::Transformation(T_B_S).inverse());
    }

    imu_rig.reset(new ze::ImuRig(T_B_Si, imus, label));
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "YAML exception during parsing: " << ex.what();
    imu_rig.reset();
    return false;
  }
  return true;
}

} // namespace YAML
