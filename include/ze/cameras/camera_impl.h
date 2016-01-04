#pragma once

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_models.h>

namespace ze {

class PinholeCamera : public Camera
{
public:
  using Scalar = typename Camera::Scalar;
  using Keypoint = typename Camera::Keypoint;
  using Bearing = typename Camera::Bearing;
  using Matrix23 = typename Camera::Matrix23;

  static constexpr size_t dimension = 4;

  using Camera::Camera;

  PinholeCamera(int width, int height, Scalar fx, Scalar fy, Scalar cx, Scalar cy)
    : Camera(width, height, CameraType::kPinhole,
             (Eigen::Matrix<Scalar, 4, 1>() << fx, fy, cx, cy).finished())
  {}

  virtual ~PinholeCamera() = default;

  virtual Bearing backProject(const Eigen::Ref<const Keypoint>& px) const override
  {
    Bearing bearing;
    internal::PinholeProjection::backProject(px.data(), this->params_.data(), bearing.data());
    return bearing;
  }

  virtual Keypoint project(const Eigen::Ref<const Bearing>& bearing) const override
  {
    Keypoint px;
    internal::PinholeProjection::project(bearing.data(), this->params_.data(), px.data());
    return px;
  }

  virtual Matrix23 dProject_dBearing(const Eigen::Ref<const Bearing>& bearing) const override
  {
    Matrix23 H;
    internal::PinholeProjection::dProject_dBearing(bearing.data(), this->params_.data(), H.data());
    return H;
  }
};

} // namespace ze
