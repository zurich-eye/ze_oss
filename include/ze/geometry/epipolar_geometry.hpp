#pragma once

#include <ze/common/transformation.h>
#include <ze/common/types.h>
#include <ze/cameras/camera_rig.h>
#include <ze/common/matrix.h>

namespace ze {

using Rect = Roi<FloatType, 2>;

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

//! \brief Compute inner
template<typename CameraModel,
         typename DistortionModel>
inline std::pair<Rect, Rect> innerAndOuterRectangles(
    const Size2u& img_size,
    Vector4& camera_parameters,
    Vector4& transformed_camera_parameters,
    Vector4& distortion_coefficients,
    Matrix3& H)
{
  constexpr int N{9};
  Matrix3X pts(3, N*N);

  int x, y, k;
  for( y = k = 0; y < N; ++y )
  {
    for( x = 0; x < N; ++x )
    {
      pts.col(k++) =
          Vector3(
            static_cast<FloatType>(x) *
              static_cast<FloatType>(img_size[0]) /
              static_cast<FloatType>(N-1),
            static_cast<FloatType>(y) *
              static_cast<FloatType>(img_size[1]) /
              static_cast<FloatType>(N-1),
            1);
    }
  }

  for (int c = 0; c < N*N; ++c)
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

  FloatType inner_x_left{-std::numeric_limits<FloatType>::max()};
  FloatType inner_x_right{std::numeric_limits<FloatType>::max()};
  FloatType inner_y_top{-std::numeric_limits<FloatType>::max()};
  FloatType inner_y_bottom{std::numeric_limits<FloatType>::max()};
  FloatType outer_x_left{std::numeric_limits<FloatType>::max()};
  FloatType outer_x_right{-std::numeric_limits<FloatType>::max()};
  FloatType outer_y_top{std::numeric_limits<FloatType>::max()};
  FloatType outer_y_bottom{-std::numeric_limits<FloatType>::max()};

  for (y = k = 0; y < N; y++)
  {
    for (x = 0; x < N; x++)
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
      if (x == N-1)
      {
        inner_x_right = std::min(inner_x_right, pt.x());
      }
      if(y == 0)
      {
        inner_y_top = std::max(inner_y_top, pt.y());
      }
      if(y == N-1)
      {
        inner_y_bottom = std::min(inner_y_bottom, pt.y());
      }
    }
  }
  Rect inner(inner_x_left, inner_y_top,
             inner_x_right - inner_x_left,
             inner_y_bottom - inner_y_top);
  Rect outer(outer_x_left, outer_y_top,
             outer_x_right - outer_x_left,
             outer_y_bottom - outer_y_top);

  return std::pair<Rect, Rect>(inner, outer);
}

template<typename CameraModel,
         typename DistortionModel>
inline void computeHorizontalStereoParameters(const Size2u& img_size,
                                              Vector4& left_camera_parameters,
                                              Vector4& left_distortion_coefficients,
                                              Vector4& right_camera_parameters,
                                              Vector4& right_distortion_coefficients,
                                              Transformation& T_L_R,
                                              Matrix3& left_H,
                                              Matrix3& right_H,
                                              Vector4& transformed_left_camera_parameters,
                                              Vector4& transformed_right_camera_parameters,
                                              float& horizontal_offset)
{
  Quaternion avg_rotation =
      Quaternion::exp(-0.5*Quaternion::log(T_L_R.getRotation()));
  Vector3 transformed_t = avg_rotation.rotate(-T_L_R.getPosition());
  Vector3 e1 = transformed_t / transformed_t.norm();
  Vector3 e2(-transformed_t(1), transformed_t(0), 0);
  e2 = e2 / e2.norm();
  Vector3 e3 = e1.cross(e2);
  Matrix3 R;
  R.row(0) = e1.transpose();
  R.row(1) = e2.transpose();
  R.row(2) = e3.transpose();

  left_H = R * avg_rotation.getRotationMatrix().transpose();
  right_H = R * avg_rotation.getRotationMatrix();

  std::cout << "left_H: " << std::endl << left_H << std::endl;
  std::cout << "right_H: " << std::endl << right_H << std::endl;
  //std::cout << "T_L_R: " << std::endl << T_L_R << std::endl;
  const Vector4* const camera_parameter_ptrs[2] = {&left_camera_parameters, &right_camera_parameters};
  const Vector4* const distortion_coefficient_ptrs[2] = {&left_distortion_coefficients, &right_distortion_coefficients};
  const Matrix3* const homography_ptrs[2] = {&left_H, &right_H};

  FloatType nx = img_size[0];
  FloatType ny = img_size[1];

  FloatType fc_new = std::numeric_limits<FloatType>::max();
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];
    // std::cout << "k: " << std::endl << camera_parameters << std::endl;
    // std::cout << "d: " << std::endl << distortion_coefficients << std::endl;
    FloatType fc = camera_parameters(1);
    if (distortion_coefficients(0) < 0)
    {
      fc *= 1 + distortion_coefficients(0)*
          (nx * nx + ny * ny) /
          (4 * fc * fc);
    }
    fc_new = std::min(fc_new, fc);
  }
  // std::cout << "fc_new: " << std::endl << fc_new << std::endl;

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
    cc_new.col(i) = Vector2((img_size[0] - 1) / 2, (img_size[1] - 1) / 2);
    cc_new.col(i) -= img_corners.block(0, 0, 2, 4).rowwise().mean();
  }
  cc_new.col(0) = cc_new.col(1) = cc_new.rowwise().mean();

  transformed_left_camera_parameters << fc_new, fc_new, cc_new(0, 0), cc_new(1, 0);
  transformed_right_camera_parameters << fc_new, fc_new, cc_new(0, 1), cc_new(1, 1);

  std::pair<Rect, Rect> left_inner_outer =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, left_camera_parameters,
        transformed_left_camera_parameters,
        left_distortion_coefficients,
        left_H);

  std::pair<Rect, Rect> right_inner_outer =
      innerAndOuterRectangles<CameraModel, DistortionModel>(
        img_size, right_camera_parameters,
        transformed_right_camera_parameters,
        right_distortion_coefficients,
        right_H);

  std::cout << "left inner rect: " << std::endl << left_inner_outer.first << std::endl;
  std::cout << "left outer rect: " << std::endl << left_inner_outer.second << std::endl;
  std::cout << "right inner rect: " << std::endl << right_inner_outer.first << std::endl;
  std::cout << "right outer rect: " << std::endl << right_inner_outer.second << std::endl;

  FloatType s0 =
      std::max(
        std::max(
          std::max(
            cc_new(0, 0)/(cc_new(0, 0) - left_inner_outer.first.x()), cc_new(1, 0)/(cc_new(1, 0) - left_inner_outer.first.y())),
          (nx - cc_new(0, 0))/(left_inner_outer.first.x() + left_inner_outer.first.width() - cc_new(0, 0))),
        (ny - cc_new(1, 0))/(left_inner_outer.first.y() + left_inner_outer.first.height() - cc_new(1, 0)));

  s0 =
      std::max(
        std::max(
          std::max(
            std::max(
              cc_new(0, 1)/(cc_new(0, 1) - right_inner_outer.first.x()), cc_new(1, 1)/(cc_new(1, 1) - right_inner_outer.first.y())),
            (nx - cc_new(0, 1))/(right_inner_outer.first.x() + right_inner_outer.first.width() - cc_new(0, 1))),
          (ny - cc_new(1, 1))/(right_inner_outer.first.y() + right_inner_outer.first.height() - cc_new(1, 1))),
        s0);

  fc_new *= s0;
  std::cout << "fc_new: " << std::endl << fc_new << std::endl;
  std::cout << "cc_new: " << std::endl << cc_new << std::endl;
}

} // namespace ze
