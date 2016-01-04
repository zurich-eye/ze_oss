#pragma once

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_models.h>

namespace ze {

class PinholeCamera : public Camera
{
public:
  using Scalar = typename Camera::Scalar;
  using Vector2 = typename Camera::Vector2;
  using Vector3 = typename Camera::Vector3;
  using Matrix23 = typename Camera::Matrix23;

  static constexpr size_t dimension = 4;

  using Camera::Camera;

  PinholeCamera(int width, int height, Scalar fx, Scalar fy, Scalar cx, Scalar cy)
    : Camera(width, height, CameraType::kPinhole,
             (Eigen::Matrix<Scalar, 4, 1>() << fx, fy, cx, cy).finished())
  {}

  virtual ~PinholeCamera() = default;

  virtual Vector3 backProject(const Eigen::Ref<const Vector2>& px) const override
  {
    CHECK_EQ(this->params_.size(), static_cast<int>(dimension));
    Vector3 bearing;
    internal::PinholeProjection::backProject(px.data(), this->params_.data(), bearing.data());
    return bearing;
  }

  virtual Vector2 project(const Eigen::Ref<const Vector3>& bearing) const override
  {
    CHECK_EQ(this->params_.size(), static_cast<int>(dimension));
    Vector2 px;
    internal::PinholeProjection::project(bearing.data(), this->params_.data(), px.data());
    return px;
  }

  virtual Matrix23 dProject_dBearing(const Eigen::Ref<const Vector3>& bearing) const override
  {
    CHECK_EQ(this->params_.size(), static_cast<int>(dimension));
    Matrix23 H;
    internal::PinholeProjection::dProject_dBearing(bearing.data(), this->params_.data(), H.data());
    return H;
  }
};

} // namespace ze
