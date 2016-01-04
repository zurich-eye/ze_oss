#pragma once

#include <string>
#include <memory>
#include <glog/logging.h>
#include <Eigen/Core>

#include <ze/common/macros.h> 

namespace ze {

enum class CameraType {
  kPinhole = 0,
  kPinholeFov = 1,
  kPinholeEquidistant = 2,
  kPinholeRadialTangential = 3
};
std::string cameraTypeString(CameraType type);

class Camera
{
public:
  ZE_POINTER_TYPEDEFS(Camera);
  using Scalar = double;
  using Vector3  = Eigen::Matrix<Scalar, 3, 1>;
  using Vector2  = Eigen::Matrix<Scalar, 2, 1>;
  using Matrix23 = Eigen::Matrix<Scalar, 2, 3>;

public:

  Camera(const int width, const int height, const CameraType type,
         const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& params);

  virtual ~Camera() = default;

  // Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& path);

  // Computes bearing vector from pixel coordinates. Z-component of returned
  // bearing vector is 1.0.
  virtual Vector3 backProject(const Eigen::Ref<const Vector2>& px) const = 0;

  // Computes pixel coordinates from bearing vector.
  virtual Vector2 project(const Eigen::Ref<const Vector3>& bearing) const = 0;

  // Computes Jacobian of projection w.r.t. bearing vector.
  virtual Matrix23 dProject_dBearing(const Eigen::Ref<const Vector3>& bearing) const = 0;

  // Print camera info.
  void print(std::ostream& out, const std::string& s = "Camera: ") const;

  // Image width in pixels.
  inline int width() const { return width_; }

  // Image height in pixels.
  inline int height() const { return height_; }

  // CameraType value representing the camera model used by the derived class.
  inline CameraType type() const { return type_; }

  // Name of the camera.
  inline const std::string& label() const { return label_; }

  // Camera parameters
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& params() const { return params_; }

  // Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  // Return if pixel u is within image boundaries.
  template<typename DerivedKeyPoint>
  bool isVisible(const Eigen::MatrixBase<DerivedKeyPoint>& px) const
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    typedef typename DerivedKeyPoint::Scalar DerivedScalar;
    return px[0] >= static_cast<DerivedScalar>(0.0)
        && px[1] >= static_cast<DerivedScalar>(0.0)
        && px[0] <  static_cast<DerivedScalar>(width_)
        && px[1] <  static_cast<DerivedScalar>(height_);
  }

  // Return if pixel u is within image boundaries with margin.
  template<typename DerivedKeyPoint>
  bool isVisibleWithMargin(
      const Eigen::MatrixBase<DerivedKeyPoint>& px,
      typename DerivedKeyPoint::Scalar margin) const
  {
    typedef typename DerivedKeyPoint::Scalar DerivedScalar;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    CHECK_LT(2 * margin, static_cast<DerivedScalar>(width_));
    CHECK_LT(2 * margin, static_cast<DerivedScalar>(height_));
    return px[0] >= margin
        && px[1] >= margin
        && px[0] < (static_cast<DerivedScalar>(width_) - margin)
        && px[1] < (static_cast<DerivedScalar>(height_) - margin);
  }

protected:

  int width_;
  int height_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> params_; // Camera parameters (fx, fy, cx, cy, distortion params...)
  std::string label_;
  CameraType type_;
};

} // namespace ze
