#pragma once

#include <string>
#include <memory>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <ze/common/transformation.h>
#include <ze/common/macros.h> 

namespace ze {

template<typename T = double>
class Camera
{
public:
  ZE_POINTER_TYPEDEFS(Camera);
  using Scalar = T;
  using Vector3 = Eigen::Matrix<Scalar, 3, 1>;
  using Vector2 = Eigen::Matrix<Scalar, 2, 1>;
  using Transformation = kindr::minimal::QuatTransformationTemplate<Scalar>;
  using Parameters = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;

  enum class Type {
    kPinhole = 0,
    kUnifiedProjection = 1,
    kOmni = 2
  };

  // Default constructor
  Camera(const int width, const int height);

  virtual ~Camera() = default;

  // Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& yaml_file);

  // Computes bearing vector f from pixel coordinates u. Z-component of the returned
  // bearing vector is 1.0. IMPORTANT: returned vector is NOT of unit length!
  virtual bool backProject3(const Eigen::Ref<const Vector2>& px, Vector3* bearing) const = 0;

  // Computes pixel coordinates u from bearing vector f.
  virtual bool project3(const Eigen::Ref<const Vector3>& bearing, Vector2* px) const = 0;

  // Print camera info
  virtual void print(std::ostream& out, const std::string& s = "Camera: ") const = 0;

  // Equivalent to focal length for projective cameras and factor for
  // omni-directional cameras that transforms small angular error to pixel-error.
  virtual double errorMultiplier() const = 0;

  virtual double getAngleError(double img_err) const = 0;

  // CameraType value representing the camera model used by the derived class.
  inline Type type() const { return camera_type_; }

  // Name of the camera.
  inline const std::string& label() const { return label_; }

  // Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  // Image width in pixels.
  int width() const { return width_; }

  // Image height in pixels.
  int height() const { return height_; }

  // Return if pixel u is within image boundaries.
  template<typename DerivedKeyPoint>
  bool isVisible(const Eigen::MatrixBase<DerivedKeyPoint>& px) const
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    typedef typename DerivedKeyPoint::Scalar DerivedScalar;
    return px[0] >= static_cast<DerivedScalar>(0.0)
        && px[1] >= static_cast<DerivedScalar>(0.0)
        && px[0] <  static_cast<DerivedScalar>(width())
        && px[1] <  static_cast<DerivedScalar>(height());
  }

  // Return if pixel u is within image boundaries with margin.
  template<typename DerivedKeyPoint>
  bool isVisibleWithMargin(
      const Eigen::MatrixBase<DerivedKeyPoint>& px,
      typename DerivedKeyPoint::Scalar margin) const
  {
    typedef typename DerivedKeyPoint::Scalar DerivedScalar;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    CHECK_LT(2 * margin, static_cast<DerivedScalar>(width()));
    CHECK_LT(2 * margin, static_cast<DerivedScalar>(height()));
    return px[0] >= margin
        && px[1] >= margin
        && px[0] < (static_cast<DerivedScalar>(width()) - margin)
        && px[1] < (static_cast<DerivedScalar>(height()) - margin);
  }

  // Set the mask. Masks must be the same size as the image and they follow the same
  // convention as OpenCV: 0 == masked, nonzero == valid.
  void setMask(const cv::Mat& mask);

  // Get the mask.
  inline const cv::Mat& getMask() const { return mask_; }

  // Clear the mask.
  inline void clearMask() { mask_ = cv::Mat(); }

  // Does the camera have a mask?
  inline bool hasMask() const { return !mask_.empty(); }

  // load from file
  void loadMask(const std::string& mask_file);

  // Check if the keypoint is masked.
  bool isMasked(const Eigen::Ref<const Vector2>& keypoint) const;

protected:
  int width_;
  int height_;
  Parameters params_; // Camera parameters (fx, fy, cx, cy, distortion params...)
  Transformation T_cam_body_;
  std::string label_;
  Type camera_type_;
  cv::Mat mask_;
};

} // namespace ze
