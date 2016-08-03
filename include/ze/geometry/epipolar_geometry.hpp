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

inline void computeHorizontalStereoParameters(Size2i img_size,
                                              Vector4& left_camera_parameters,
                                              Vector4& left_distortion_coefficients,
                                              Vector4& right_camera_parameters,
                                              Vector4& right_distortion_coefficients,
                                              Transformation T_L_R,
                                              Matrix3 left_H,
                                              Matrix3 right_H,
                                              Vector4 transformed_left_camera_parameters,
                                              Vector4 transformed_right_camera_parameters,
                                              float& horizontal_offset)
{
  Quaternion avg_rotation = Quaternion::exp(-0.5*Quaternion::log(T_L_R.getRotation()));
}

} // namespace ze
