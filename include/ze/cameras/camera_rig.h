#pragma once

#include <string>
#include <vector>

#include <ze/common/types.h>
#include <ze/common/macros.h>
#include <ze/common/transformation.h>

namespace ze {

class Camera;
using CameraVector = std::vector<std::shared_ptr<Camera>>;

class CameraRig
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ZE_POINTER_TYPEDEFS(CameraRig);

  CameraRig(
      const TransformationVector& T_C_B,
      const CameraVector& cameras,
      const std::string& label);

  //! Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
  static CameraRig::Ptr loadFromYaml(const std::string& yaml_file);

  //!@{
  //! Camera poses with respect to body frame.

  //! Get the pose of body frame with respect to the camera i.
  inline const Transformation& get_T_C_B(size_t camera_index) const
  {
    return T_C_B_.at(camera_index);
  }

  //! Get all transformations.
  inline const TransformationVector& getTransformationVector() const
  {
    return T_C_B_;
  }
  //!@}

  //!@{
  //! Camera accessors:

  //! Get camera i.
  inline const Camera& at(size_t camera_index) const
  {
    return *cameras_.at(camera_index);
  }

  //! Get camera i.
  inline std::shared_ptr<Camera> atShared(size_t camera_index)
  {
    return cameras_.at(camera_index);
  }

  //! Get camera i.
  inline std::shared_ptr<const Camera> atShared(size_t camera_index) const
  {
    return cameras_.at(camera_index);
  }

  //! Get all cameras.
  inline const CameraVector& getCameraVector() const
  {
    return cameras_;
  }
  //!@}

  //! Get the number of cameras in rig.
  inline size_t size() const
  {
    return cameras_.size();
  }

  //! Get of the camera rig.
  inline const std::string& getLabel() const
  {
    return label_;
  }

  //! Print camera infos
  void print(std::ostream& out, const std::string& s = "CameraRig:") const;

private:

  //! Default constructor not available.
  CameraRig() = default;

  //! The mounting transformations.
  TransformationVector T_C_B_;

  //! The camera geometries.
  CameraVector cameras_;

  //! A label for this camera rig, a name.
  std::string label_;
};

} // namespace ze

