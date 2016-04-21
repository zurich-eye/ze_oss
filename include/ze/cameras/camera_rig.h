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

  //! Default constructor not available.
  CameraRig() = delete;

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
  inline const CameraVector& getCameraVector() const { return cameras_; }
  //!@}

  //! Get the number of cameras in rig.
  inline size_t size() const { return cameras_.size(); }

  //! Get label of rig.
  inline const std::string& getLabel() const { return label_; }

  //!@{
  //! Camera iteration:
  typedef CameraVector::value_type value_type;
  typedef CameraVector::iterator iterator;
  typedef CameraVector::const_iterator const_iterator;
  CameraVector::iterator begin() { return cameras_.begin(); }
  CameraVector::iterator end() { return cameras_.end(); }
  CameraVector::const_iterator begin() const { return cameras_.begin(); }
  CameraVector::const_iterator end() const { return cameras_.end(); }
  CameraVector::const_iterator cbegin() const { return cameras_.cbegin(); }
  CameraVector::const_iterator cend() const { return cameras_.cend(); }
  //!@}

private:

  //! The mounting transformations.
  TransformationVector T_C_B_;

  //! The camera geometries.
  CameraVector cameras_;

  //! A label for this camera rig, a name.
  std::string label_;
};

std::ostream& operator<<(std::ostream& out, const CameraRig& rig);

} // namespace ze

