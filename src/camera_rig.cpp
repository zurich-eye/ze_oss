#include <ze/cameras/camera_rig.h>
#include <ze/cameras/camera_utils.h>
#include <ze/cameras/camera_yaml_serialization.h>
#include <ze/common/path_utils.h>

namespace ze {

// -----------------------------------------------------------------------------
CameraRig::CameraRig(
    const TransformationVector& T_C_B,
    const CameraVector& cameras,
    const std::string& label,
    const FloatType stereo_min_fov_overlap,
    const FloatType stereo_min_baseline)
  : T_C_B_(T_C_B)
  , cameras_(cameras)
  , label_(label)
{
  CHECK_EQ(T_C_B_.size(), cameras_.size());
  for(size_t i = 0; i < size(); ++i)
  {
    CHECK_NOTNULL(cameras_[i].get());
  }

  if (size() > 1u)
  {
    setStereoPairs(identifyStereoPairsInRig(
                     *this, stereo_min_fov_overlap, stereo_min_baseline));
  }
}

// -----------------------------------------------------------------------------
CameraRig::Ptr CameraRig::loadFromYaml(const std::string& yaml_file)
{
  CHECK(fileExists(yaml_file)) << "File does not exist: " << yaml_file;

  CameraRig::Ptr rig;
  try
  {
    YAML::Node doc = YAML::LoadFile(yaml_file.c_str());
    rig = doc.as<CameraRig::Ptr>();
  }
  catch (const std::exception& ex)
  {
    LOG(ERROR) << "Cannot load CameraRig from file:" << yaml_file << "\n"
               << ex.what();
    return nullptr;
  }

  return rig;
}

// -----------------------------------------------------------------------------
CameraRig::Ptr CameraRig::getSubRig(
    const std::vector<uint32_t>& camera_indices,
    const std::string& label)
{
  CameraVector cameras;
  TransformationVector T;
  for (uint32_t i : camera_indices)
  {
    cameras.push_back(atShared(i));
    T.push_back(T_C_B(i));
  }
  return std::make_shared<CameraRig>(T, cameras, label);
}

// -----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& out, const CameraRig& rig)
{
  out << "Camera Rig: \n"
      << "  Label = " << rig.label() << "\n"
      << "  Stereo pairs =" << rig.stereoPairs() << "\n";
  for (size_t i = 0; i < rig.size(); ++i)
  {
    out << "- Camera " << i << "\n"
        << rig.at(i) << "\n"
        << "    T_B_C = \n" << rig.T_C_B(i).inverse() << "\n";
  }
  return out;
}

// -----------------------------------------------------------------------------
std::ostream& operator<<(std::ostream& out, const StereoIndexPairs& stereo_pairs)
{
  for (auto it : stereo_pairs)
  {
    out << " (" << static_cast<int>(it.first) << ", "
        << static_cast<int>(it.second) << ")";
  }
  return out;
}

// -----------------------------------------------------------------------------
StereoIndexPairs identifyStereoPairsInRig(
    const CameraRig& rig,
    const FloatType& min_fov_overlap,
    const FloatType& min_baseline)
{
  StereoIndexPairs pairs;
  for (uint32_t cam_A = 0u; cam_A < rig.size(); ++cam_A)
  {
    for (uint32_t cam_B = cam_A + 1u; cam_B < rig.size(); ++cam_B)
    {
      FloatType overlap = overlappingFieldOfView(rig, cam_A, cam_B);
      FloatType baseline = (rig.T_C_B(cam_B) * rig.T_C_B(cam_A).inverse()).getPosition().norm();
      VLOG(10) << "Camera " << cam_A << " and " << cam_B << ": Overlap = "
               << overlap << ", Baseline = " << baseline;

      if (overlap > min_fov_overlap && baseline > min_baseline)
      {
        pairs.push_back(std::make_pair(cam_A, cam_B));
      }
    }
  }
  return pairs;
}

} // namespace ze
