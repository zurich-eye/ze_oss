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

  //std::cout << "left_H: " << std::endl << left_H << std::endl;
  //std::cout << "right_H: " << std::endl << right_H << std::endl;
  //std::cout << "T_L_R: " << std::endl << T_L_R << std::endl;
  const Vector4* camera_parameter_ptrs[2] = {&left_camera_parameters, &right_camera_parameters};
  const Vector4* distortion_coefficient_ptrs[2] = {&left_distortion_coefficients, &right_distortion_coefficients};

  double nx = img_size[0];
  double ny = img_size[1];

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
  std::cout << "fc_new: " << std::endl << fc_new << std::endl;

  Matrix22 cc_new;
  for (int8_t i = 0; i < 2; ++i)
  {
    const Vector4& camera_parameters = *camera_parameter_ptrs[i];
    const Vector4& distortion_coefficients = *distortion_coefficient_ptrs[i];

    Matrix34 img_corners;
    img_corners << 0, nx, nx, 0,
        0, 0, ny, ny,
        1, 1, 1, 1;

    Vector4 temp_left_cam_params;
    temp_left_cam_params << fc_new, fc_new, 0, 0;
    for (int8_t c = 0; c < 4; ++c)
    {
      CameraModel::backProject(camera_parameters.data(), img_corners.col(c).data());
      DistortionModel::undistort(distortion_coefficients.data(), img_corners.col(c).data());
      img_corners.col(c) = left_H * img_corners.col(c);
      img_corners.col(c) /= img_corners.col(c)(2);
      CameraModel::project(temp_left_cam_params.data(), img_corners.col(c).data());
    }
    // std::cout << img_corners << std::endl;

    cc_new.col(i) = Vector2((img_size[0] - 1) / 2, (img_size[1] - 1) / 2);
    cc_new.col(i) -= img_corners.block(0, 0, 2, 4).rowwise().mean();

   // std::cout << "cc_new: " << std::endl << cc_new.col(i) << std::endl;

  }
  //std::cout << "avg cc_new: " << std::endl << cc_new.rowwise().sum() * 0.5 << std::endl;
}

} // namespace ze
