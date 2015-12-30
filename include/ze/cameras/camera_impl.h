#pragma once

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_models.h>

namespace ze {

template<typename T>
class PinholeCamera : public Camera<T>
{
public:
  using Scalar = T;
  using Vector2 = typename Camera<Scalar>::Vector2;
  using Vector3 = typename Camera<Scalar>::Vector3;
  using Matrix23 = typename Camera<Scalar>::Matrix23;

  static constexpr size_t dimension = 4;

  PinholeCamera(int width, int height, T fx, T fy, T cx, T cy)
    : Camera<Scalar>(width, height, CameraType::kPinhole)
  {
    this->params_.resize(dimension, 1);
    this->params_ << fx, fy, cx, cy;
  }

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
