#pragma once

#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/cameras/camera_rig.h>
#include <ze/common/matrix.h>

namespace ze {

// ----------------------------------------------------------------------------
//! Compute essential matrix from given camera transformation
inline Matrix3 essentialMatrix(const Transformation& T)
{
  return skewSymmetric(T.getPosition()) * T.getRotationMatrix();
}

// ----------------------------------------------------------------------------
//! Compute fundamental matrix from given camera transformation
inline Matrix3 fundamentalMatrix(const Transformation& T_cam0_cam1,
                                 const VectorX& projection_parameters0,
                                 const VectorX& projection_parameters1)
{
  CHECK_EQ(projection_parameters0.size(), 4u);
  CHECK_EQ(projection_parameters1.size(), 4u);

  Matrix3 K0, K1;
  K0 << projection_parameters0(0), 0, projection_parameters0(2),
      0, projection_parameters0(1), projection_parameters0(3),
      0, 0, 1;
  K1 << projection_parameters1(0), 0, projection_parameters1(2),
      0, projection_parameters1(1), projection_parameters1(3),
      0, 0, 1;

  return (K0.inverse().transpose() * essentialMatrix(T_cam0_cam1) * K1.inverse());
}

// ----------------------------------------------------------------------------
//! Stereo Rectification

using Rect = Roi<FloatType, 2>;

//! \brief Compute inner and outer rectangles.
//!
//! The inner rectangle is inscribed in the undistorted-rectified image.
//! The outer rectangle is circumscribed about the undistorted-rectified image.
//! \param img_size The size of the original (distorted) image.
//! \param camera_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the original image.
//! \param transformed_camera_parameters Vector of intrinsic parameters [fx', fy', cx', cy']
//! for the undistorted-rectified image.
//! \param distortion_coefficients Vector of distortion coefficients.
//! \param H Rectifying homography matrix.
//! \return A pair containing the rectangles inner (first) and outer (second).
template<typename CameraModel,
         typename DistortionModel>
inline std::pair<Rect, Rect> innerAndOuterRectangles(
    const Size2u& img_size,
    const Vector4& camera_parameters,
    const Vector4& transformed_camera_parameters,
    const Vector4& distortion_coefficients,
    const Matrix3& H)
{
  //! Sample the image in num_pts*num_pts locations
  constexpr int num_pts{9}; //!< Number of sampling point for each image dimension
  Matrix3X pts(3, num_pts*num_pts);     //!< Sampling points

  // int x, y, k;
  for (int y = 0, k = 0; y < num_pts; ++y)
  {
    for (int x = 0; x < num_pts; ++x)
    {
      pts.col(k++) =
          Vector3(
            static_cast<FloatType>(x) *
            static_cast<FloatType>(img_size[0]) /
          static_cast<FloatType>(num_pts-1),
          static_cast<FloatType>(y) *
          static_cast<FloatType>(img_size[1]) /
          static_cast<FloatType>(num_pts-1),
          1);
    }
  }

  //! For every sampling point (u,v) compute the corresponding (u', v')
  //! in the undistorted-rectified image:
  //! x" = (u - cx)/fx
  //! y" = (v - cy)/fy
  //! (x', y') = undistort(distortion_coefficients, (x", y"))
  //! [X Y W]^T = H*[x' y' 1]^T
  //! x = X / W, y = Y / W
  //! u' = x * fx' + cx'
  //! v' = y * fy' + cy'
  for (int c = 0; c < num_pts * num_pts; ++c)
  {
    CameraModel::backProject(camera_parameters.data(),
                             pts.col(c).data());
    DistortionModel::undistort(distortion_coefficients.data(),
                               pts.col(c).data());
    pts.col(c) = H * pts.col(c);
    pts.col(c) /= pts.col(c)(2);
    CameraModel::project(transformed_camera_parameters.data(),
                         pts.col(c).data());
  }

  //! Rectangles are specified by two points.
  FloatType inner_x_left{-std::numeric_limits<FloatType>::max()};
  FloatType inner_x_right{std::numeric_limits<FloatType>::max()};
  FloatType inner_y_top{-std::numeric_limits<FloatType>::max()};
  FloatType inner_y_bottom{std::numeric_limits<FloatType>::max()};
  FloatType outer_x_left{std::numeric_limits<FloatType>::max()};
  FloatType outer_x_right{-std::numeric_limits<FloatType>::max()};
  FloatType outer_y_top{std::numeric_limits<FloatType>::max()};
  FloatType outer_y_bottom{-std::numeric_limits<FloatType>::max()};

  //! Iterate over the sampling points and adjust the rectangle bounds.
  for (int y = 0, k = 0; y < num_pts; y++)
  {
    for (int x = 0; x < num_pts; x++)
    {
      const Vector3& pt = pts.col(k++);
      outer_x_left = std::min(outer_x_left, pt.x());
      outer_x_right = std::max(outer_x_right, pt.x());
      outer_y_top = std::min(outer_y_top, pt.y());
      outer_y_bottom = std::max(outer_y_bottom, pt.y());

      if (x == 0)
      {
        inner_x_left = std::max(inner_x_left, pt.x());
      }
      if (x == num_pts - 1)
      {
        inner_x_right = std::min(inner_x_right, pt.x());
      }
      if (y == 0)
      {
        inner_y_top = std::max(inner_y_top, pt.y());
      }
      if (y == num_pts - 1)
      {
        inner_y_bottom = std::min(inner_y_bottom, pt.y());
      }
    }
  }

  //! Compute and return the rectangles.
  Rect inner(inner_x_left, inner_y_top,
             inner_x_right - inner_x_left,
             inner_y_bottom - inner_y_top);
  Rect outer(outer_x_left, outer_y_top,
             outer_x_right - outer_x_left,
             outer_y_bottom - outer_y_top);

  return std::pair<Rect, Rect>(inner, outer);
}

//!\brief Compute rectification parameters for a horizontal stereo pair
//!
//! The function is specific for the horizontal-stereo case.
//! \param img_size The size of the original (distorted) image.
//! \param left_camera_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the left camera of the stereo pair.
//! \param left_distortion_coefficients Vector of distortion coefficients
//! for the left camera of the stereo pair.
//! \param right_camera_parameters Vector of intrinsic parameters [fx, fy, cx, cy]
//! for the right camera of the stereo pair.
//! \param right_distortion_coefficients Vector of distortion coefficients
//! for the left camera of the stereo pair.
//! \param T_L_R Stereo extrinsic parameters, i.e. the transformation right-to-left.
//! \param left_H Output rectifying homography for the left camera.
//! \param right_H Output rectifying homography for the right camera.
//! \param transformed_left_camera_parameters Output transformed parameters for the left camera.
//! \param transformed_right_camera_parameters Output transformed parameters for the right camera.
//! \param horizontal_offset Output displacement for the rectified stereo pair.
template<typename CameraModel,
         typename DistortionModel>
inline std::tuple<Matrix3, Matrix3, Vector4, Vector4, FloatType>
computeHorizontalStereoParameters(const Size2u& img_size,
                                  const Vector4& left_camera_parameters,
                                  const Vector4& left_distortion_coefficients,
                                  const Vector4& right_camera_parameters,
                                  const Vector4& right_distortion_coefficients,
                                  const Transformation& T_L_R)
{
  //! Compute the recification homographies as in
  //! Trucco, Verry: Introductory techniques for 3D computer vision,
  //! Prentice Hall 1998, page 160.
  const Quaternion avg_rotation =
      Quaternion::exp(-0.5*Quaternion::log(T_L_R.getRotation()));
  const Vector3 transformed_t = avg_rotation.rotate(-T_L_R.getPosition());
  const Vector3 e1 = transformed_t / transformed_t.norm();
  Vector3 e2(-transformed_t(1), transformed_t(0), 0);
  e2 = e2 / e2.norm();
  const Vector3 e3 = e1.cross(e2);
  Matrix3 rect_R;
  rect_R.row(0) = e1.transpose();
  rect_R.row(1) = e2.transpose();
  rect_R.row(2) = e3.transpose();

  //! Rotate both cameras according to the average rotation.
  const Matrix3 left_H = rect_R * avg_rotation.getRotationMatrix().transpose();
  const Matrix3 right_H = rect_R * avg_rotation.getRotationMatrix();

  //! The images rectified according to left_H and right_H will not be contained
  //! in the same region of the image plane as the original image.
  //! Here we alter the focal lengths and the principal point to keep
  //! all points within the original image size.
  const Vector4* const camera_parameter_ptrs[2] = {&left_camera_parameters,
                                                   &right_camera_parameters};
  const Vector4* const distortion_coefficient_ptrs[2] = {&left_distortion_coefficients,
                                                         &right_distortion_coefficients};
  const Matrix3* const homography_ptrs[2] = {&left_H,
                                             &right_H};

  const FloatType nx = img_size[0];
  const FloatType ny = img_size[1];

  FloatType fc_new = std::numeric_limits<FloatType>::max();
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];
    FloatType fc = camera_parameters(1);
    if (distortion_coefficients(0) < 0)
    {
      fc *= 1 + distortion_coefficients(0)*
          (nx * nx + ny * ny) /
          (4 * fc * fc);
    }
    fc_new = std::min(fc_new, fc);
  }

  Matrix22 cc_new;
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];
    const Matrix3& H = *homography_ptrs[i];

    Matrix34 img_corners;
    img_corners << 0, nx, nx, 0,
        0, 0, ny, ny,
        1, 1, 1, 1;

    Vector4 temp_cam_params;
    temp_cam_params << fc_new, fc_new, 0, 0;
    for (int8_t c = 0; c < 4; ++c)
    {
      CameraModel::backProject(camera_parameters.data(), img_corners.col(c).data());
      DistortionModel::undistort(distortion_coefficients.data(), img_corners.col(c).data());
      img_corners.col(c) = H * img_corners.col(c);
      img_corners.col(c) /= img_corners.col(c)(2);
      CameraModel::project(temp_cam_params.data(), img_corners.col(c).data());
    }
    cc_new.col(i) = Vector2((nx - 1) / 2, (ny - 1) / 2);
    cc_new.col(i) -= img_corners.block(0, 0, 2, 4).rowwise().mean();
  }
  cc_new.col(0) = cc_new.col(1) = cc_new.rowwise().mean();

  Vector4 transformed_left_camera_parameters;
  Vector4 transformed_right_camera_parameters;
  transformed_left_camera_parameters << fc_new, fc_new, cc_new(0, 0), cc_new(1, 0);
  transformed_right_camera_parameters << fc_new, fc_new, cc_new(0, 1), cc_new(1, 1);

  std::pair<Rect, Rect> l_rects =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, left_camera_parameters,
        transformed_left_camera_parameters,
        left_distortion_coefficients,
        left_H);

  std::pair<Rect, Rect> r_rects =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, right_camera_parameters,
        transformed_right_camera_parameters,
        right_distortion_coefficients,
        right_H);

  //! @todo (MPI) support new image size.
  //! @todo (MPI) support different scales in [0, 1].
  //! Currently only scale = 0 is supported (s0).

  FloatType s0 =
      std::max(
        std::max(
          std::max(
            cc_new(0, 0)/(cc_new(0, 0) - l_rects.first.x()),
            cc_new(1, 0)/(cc_new(1, 0) - l_rects.first.y())),
          (nx - cc_new(0, 0))/(l_rects.first.x() + l_rects.first.width() - cc_new(0, 0))),
        (ny - cc_new(1, 0))/(l_rects.first.y() + l_rects.first.height() - cc_new(1, 0)));

  s0 = std::max(
        std::max(
          std::max(
            std::max(
              cc_new(0, 1)/(cc_new(0, 1) - r_rects.first.x()),
              cc_new(1, 1)/(cc_new(1, 1) - r_rects.first.y())),
            (nx - cc_new(0, 1))/(r_rects.first.x() + r_rects.first.width() - cc_new(0, 1))),
          (ny - cc_new(1, 1))/(r_rects.first.y() + r_rects.first.height() - cc_new(1, 1))),
        s0);

  transformed_left_camera_parameters(0) =
      transformed_left_camera_parameters(1) =
      transformed_right_camera_parameters(0) =
      transformed_right_camera_parameters(1) = fc_new * s0;

  const FloatType horizontal_offset = transformed_t(0) * s0;

  return std::make_tuple(left_H,
                         right_H,
                         transformed_left_camera_parameters,
                         transformed_right_camera_parameters,
                         horizontal_offset);
}

} // namespace ze
