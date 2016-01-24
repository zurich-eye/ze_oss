#pragma once

#include <string>
#include <memory>
#include <glog/logging.h>
#include <Eigen/Core>

#include <ze/common/macros.h> 
#include <ze/common/types.h>

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

public:

  Camera(const int width, const int height, const CameraType type,
         const Vector& projection_params, const Vector& distortion_params);

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
  virtual Eigen::Matrix<FloatType, 6, Eigen::Dynamic>
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
  inline const Eigen::Matrix<FloatType, Eigen::Dynamic, 1>& params() const { return projection_params_; }

  //! Set user-specific camera label.
  inline void setLabel(const std::string& label) { label_ = label; }

  //! Return if pixel u is within image boundaries.
  template<typename DerivedKeyPoint>
  bool isVisible(const Eigen::MatrixBase<DerivedKeyPoint>& px) const
  {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    typedef typename DerivedKeyPoint::FloatType DerivedFloatType;
    return px[0] >= static_cast<DerivedFloatType>(0.0)
        && px[1] >= static_cast<DerivedFloatType>(0.0)
        && px[0] <  static_cast<DerivedFloatType>(width_)
        && px[1] <  static_cast<DerivedFloatType>(height_);
  }

  //! Return if pixel u is within image boundaries with margin.
  template<typename DerivedKeyPoint>
  bool isVisibleWithMargin(
      const Eigen::MatrixBase<DerivedKeyPoint>& px,
      typename DerivedKeyPoint::FloatType margin) const
  {
    typedef typename DerivedKeyPoint::FloatType DerivedFloatType;
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(DerivedKeyPoint, 2, 1);
    CHECK_LT(2 * margin, static_cast<DerivedFloatType>(width_));
    CHECK_LT(2 * margin, static_cast<DerivedFloatType>(height_));
    return px[0] >= margin
        && px[1] >= margin
        && px[0] < (static_cast<DerivedFloatType>(width_) - margin)
        && px[1] < (static_cast<DerivedFloatType>(height_) - margin);
  }

protected:

  int width_;
  int height_;

  //! Camera projection parameters, e.g., (fx, fy, cx, cy).
  Vector projection_params_;

  //! Camera distortion parameters, e.g., (k1, k2, r1, r2).
  Vector distortion_params_;

  //! Camera distortion parameters
  std::string label_;
  CameraType type_;
};

} // namespace ze
