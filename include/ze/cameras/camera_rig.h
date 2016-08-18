#pragma once

#include <string>
#include <vector>
#include <gflags/gflags.h>

#include <ze/cameras/camera.h>
#include <ze/common/types.h>
#include <ze/common/macros.h>
#include <ze/common/transformation.h>

DECLARE_string(calib_filename);
DECLARE_string(mask_cam0);
DECLARE_string(mask_cam1);
DECLARE_string(mask_cam2);
DECLARE_string(mask_cam3);
DECLARE_bool(calib_use_single_camera);

namespace ze {

// convenience typedefs.
using CameraVector     = std::vector<Camera::Ptr>;
using StereoIndexPair  = std::pair<uint8_t, uint8_t>;
using StereoIndexPairs = std::vector<StereoIndexPair>;

class CameraRig
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(CameraRig);

  CameraRig() = delete;

  CameraRig(
      const TransformationVector& T_C_B,
      const CameraVector& cameras,
      const std::string& label,
      const FloatType stereo_min_fov_overlap = 0.7,
      const FloatType stereo_min_baseline = 0.04);

  //! @name Camera extrinsic calibration.
  //! @{
  inline const Transformation& T_C_B(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, T_C_B_.size());
    return T_C_B_[camera_index];
  }

  inline Transformation T_B_C(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, T_C_B_.size());
    return T_C_B_[camera_index].inverse();
  }

  inline const TransformationVector& T_C_B_vec() const
  {
    return T_C_B_;
  }
  //! @}

  //! @name Camera accessors.
  //! @{
  inline const Camera& at(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return *cameras_[camera_index];
  }

  inline std::shared_ptr<Camera> atShared(size_t camera_index)
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return cameras_[camera_index];
  }

  inline std::shared_ptr<const Camera> atShared(size_t camera_index) const
  {
    DEBUG_CHECK_LT(camera_index, cameras_.size());
    return cameras_[camera_index];
  }

  inline const CameraVector& cameras() const { return cameras_; }
  //! @}

  inline size_t size() const { return cameras_.size(); }

  inline const std::string& label() const { return label_; }

  inline const StereoIndexPairs& stereoPairs() const { return stereo_pairs_; }

  inline void setStereoPairs(const StereoIndexPairs& pairs) { stereo_pairs_ = pairs; }

  //! @name Camera iteration.
  //! @{
  typedef CameraVector::value_type value_type;
  typedef CameraVector::iterator iterator;
  typedef CameraVector::const_iterator const_iterator;
  CameraVector::iterator begin() { return cameras_.begin(); }
  CameraVector::iterator end() { return cameras_.end(); }
  CameraVector::const_iterator begin() const { return cameras_.begin(); }
  CameraVector::const_iterator end() const { return cameras_.end(); }
  CameraVector::const_iterator cbegin() const { return cameras_.cbegin(); }
  CameraVector::const_iterator cend() const { return cameras_.cend(); }
  //! @}

  //! Get a rig that contains only a subset of the cameras.
  CameraRig::Ptr getSubRig(
      const std::vector<uint32_t>& camera_indices,
      const std::string& label);

private:
  //! The mounting transformations.
  TransformationVector T_C_B_;

  //! The camera geometries.
  CameraVector cameras_;

  //! Unique pairs of camera indices with overlapping field of view.
  StereoIndexPairs stereo_pairs_;

  //! A label for this camera rig, a name.
  std::string label_;
};

//! Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
CameraRig::Ptr cameraRigFromYaml(const std::string& yaml_file);

//! Load a camera rig form a gflag parameters. Returns a nullptr if the loading fails.
CameraRig::Ptr cameraRigFromGflags();

//! Formatted printing.
std::ostream& operator<<(std::ostream& out, const CameraRig& rig);
std::ostream& operator<<(std::ostream& out, const StereoIndexPairs& stereo_pairs);

//! Compute overlapping field of view and baseline for all stereo pairs
StereoIndexPairs identifyStereoPairsInRig(
    const CameraRig& rig,
    const FloatType& min_fov_overlap,
    const FloatType& min_baseline);

} // namespace ze

