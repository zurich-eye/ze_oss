#pragma once

#include <ze/cameras/camera.h>
#include <ze/cameras/camera_models.h>

namespace ze {

template<typename T>
class PinholeCamera : public Camera<T>
{
public:
  using Scalar = T;
  using Vector2 = typename Camera<T>::Vector2;
  using Vector3 = typename Camera<T>::Vector3;
  static constexpr size_t dimension = 4;

  PinholeCamera(int width, int height, T fx, T fy, T cx, T cy)
    : Camera<Scalar>(width, height, CameraType::kPinhole)
  {
    this->params_.resize(dimension, 1);
    this->params_ << fx, fy, cx, cy;
  }

  virtual ~PinholeCamera() = default;

  // Computes bearing vector bearing from pixel coordinates px. Z-component of
  // the returned bearing vector is 1.0.
  virtual void backProject(const Eigen::Ref<const Vector2>& px, Vector3* bearing) const override
  {
    internal::PinholeProjection::backProject(px.data(), this->params_.data(), bearing->data());
  }

  // Computes pixel coordinates px from bearing vector bearing.
  virtual void project(const Eigen::Ref<const Vector3>& bearing, Vector2* px) const override
  {
    internal::PinholeProjection::project(bearing.data(), this->params_.data(), px->data());
  }
};

} // namespace ze
