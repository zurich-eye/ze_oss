#pragma once

#include <string>
#include <memory>
#include <glog/logging.h>
#include <Eigen/Core>

#include <ze/common/macros.h> 

namespace ze {

enum class CameraType {
  Pinhole = 0,
  PinholeFov = 1,
  PinholeEquidistant = 2,
  PinholeRadialTangential = 3
};

class Camera
{
public:
  ZE_POINTER_TYPEDEFS(Camera);
  using Scalar = double;
  using Bearing  = Eigen::Matrix<Scalar, 3, 1>;
  using Position  = Eigen::Matrix<Scalar, 3, 1>;
  using Keypoint  = Eigen::Matrix<Scalar, 2, 1>;
  using Bearings  = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
  using Positions  = Eigen::Matrix<Scalar, 3, Eigen::Dynamic>;
  using Keypoints  = Eigen::Matrix<Scalar, 2, Eigen::Dynamic>;
  using Matrix23 = Eigen::Matrix<Scalar, 2, 3>;

public:

  Camera(const int width, const int height, const CameraType type,
         const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& params);

  virtual ~Camera() = default;

  //! Load a camera rig form a yaml file. Returns a nullptr if the loading fails.
  static Ptr loadFromYaml(const std::string& path);

  //! Vearing vector from pixel coordinates. Z-component of return value is 1.0.
  virtual Bearing backProject(const Eigen::Ref<const Keypoint>& px) const = 0;

  //! Computes pixel coordinates from bearing vector.
  virtual Keypoint project(const Eigen::Ref<const Bearing>& bearing) const = 0;

  //! Computes Jacobian of projection w.r.t. bearing vector.
  virtual Matrix23 dProject_dLandmark(const Eigen::Ref<const Position>& pos) const = 0;

  //!@{
  //! Block operations: Always prefer these functions to avoid cache misses.

  //! Back projects a block of keypoints.
  virtual Bearings backProjectVectorized(const Keypoints& px_vec) const;

  //! Projects a block of bearing vectors.
  virtual Keypoints projectVectorized(const Bearings& bearing_vec) const;

  //! Vectorized computation of projection Jacobian. Column-wise reshaped.
  virtual Eigen::Matrix<Scalar, 6, Eigen::Dynamic>
  dProject_dLandmarkVectorized(const Positions& pos_vec) const;
  //!@}

  //! Print camera info.
  void print(std::ostream& out, const std::string& s = "Camera: ") const;

  //! Image width in pixels.
  inline int width() const { return width_; }

  //! Image height in pixels.
  inline int height() const { return height_; }

  //! CameraType value representing the camera model used by the derived class.
  inline CameraType type() const { return type_; }

  //! CameraType as descriptive string.
  std::string typeString() const;

  //! Name of the camera.
  inline const std::string& label() const { return label_; }

  //! Camera parameters
  inline const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& params() const { return params_; }

  //! Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  //! Return if pixel u is within image boundaries.
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

  //! Return if pixel u is within image boundaries with margin.
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
