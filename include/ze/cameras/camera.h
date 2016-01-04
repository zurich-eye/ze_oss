#pragma once

#include <string>
#include <memory>
#include <glog/logging.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ze/common/macros.h> 
#include <ze/cameras/camera_yaml_serialization.h>

namespace ze {

enum class CameraType {
  kPinhole = 0,
  kPinholeFov = 1,
  kPinholeEquidistant = 2,
  kPinholeRadialTangential = 3
};
std::string cameraTypeString(CameraType type);

template<typename T>
class Camera
{
public:
  ZE_POINTER_TYPEDEFS(Camera);
  using Scalar = T;
  using Vector3  = Eigen::Matrix<Scalar, 3, 1>;
  using Vector2  = Eigen::Matrix<Scalar, 2, 1>;
  using Matrix23 = Eigen::Matrix<Scalar, 2, 3>;

  // Default constructor
  Camera(const int width, const int height, const CameraType type)
  : width_(width)
  , height_(height)
  , type_(type)
  {}

  virtual ~Camera() = default;

  // Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& path)
  {
    try
    {
      YAML::Node doc = YAML::LoadFile(path.c_str());
      return doc.as<Camera::Ptr>();
    }
    catch (const std::exception& ex)
    {
      LOG(ERROR) << "Failed to load Camera from file " << path << " with the error: \n"
                 << ex.what();
    }
    return Camera<T>::Ptr();
  }

  // Computes bearing vector from pixel coordinates. Z-component of returned
  // bearing vector is 1.0.
  virtual Vector3 backProject(const Eigen::Ref<const Vector2>& px) const = 0;

  // Computes pixel coordinates from bearing vector.
  virtual Vector2 project(const Eigen::Ref<const Vector3>& bearing) const = 0;

  // Computes Jacobian of projection w.r.t. bearing vector:
  virtual Matrix23 dProject_dBearing(const Eigen::Ref<const Vector3>& bearing) const = 0;

  // Print camera info
  void print(std::ostream& out, const std::string& s = "Camera: ") const
  {
    out << s << std::endl
        << "  Label = " << label_ << std::endl
        << "  Model = " << cameraTypeString(type_) << std::endl
        << "  Dimensions = " << width_ << "x" << height_ << std::endl
        << "  Parameters = " << params_.transpose() << std::endl;
  }

  // CameraType value representing the camera model used by the derived class.
  inline CameraType type() const { return type_; }

  // Name of the camera.
  inline const std::string& label() const { return label_; }

  // Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  // Image width in pixels.
  inline int width() const { return width_; }

  // Image height in pixels.
  inline int height() const { return height_; }

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
  void setMask(const cv::Mat& mask)
  {
    CHECK_EQ(height_, mask.rows);
    CHECK_EQ(width_, mask.cols);
    CHECK_EQ(mask.type(), CV_8UC1);
    mask_ = mask;
  }

  // Get the mask.
  inline const cv::Mat& getMask() const { return mask_; }

  // Clear the mask.
  inline void clearMask() { mask_ = cv::Mat(); }

  // Does the camera have a mask?
  inline bool hasMask() const { return !mask_.empty(); }

  // load from file
  void loadMask(const std::string& mask_file)
  {
    cv::Mat mask(cv::imread(mask_file, 0));
    if(mask.data)
      setMask(mask);
    else
      LOG(FATAL) << "Unable to load mask file.";
  }

  // Check if the keypoint is masked.
  bool isMasked(const Eigen::Ref<const Vector2>& px) const
  {
    return px[0] < 0.0 ||
           px[0] >= static_cast<T>(width_) ||
           px[1] < 0.0 ||
           px[1] >= static_cast<T>(height_) ||
           (!mask_.empty() &&
             mask_.at<uint8_t>(static_cast<int>(px[1]), static_cast<int>(px[0])) == 0);
  }

protected:
  int width_;
  int height_;
  CameraType type_;
  Eigen::Matrix<Scalar, Eigen::Dynamic, 1> params_; // Camera parameters (fx, fy, cx, cy, distortion params...)
  std::string label_;
  cv::Mat mask_;
};

} // namespace ze
