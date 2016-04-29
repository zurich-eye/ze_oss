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
  return ze::skewSymmetric(T.getPosition()) * T.getRotationMatrix();
}

// ----------------------------------------------------------------------------
//! Compute fundamental matrix from given camera transformation
//! @note Function assumes a camera rig with two cameras and cam0 recording the
//!       reference image and cam1 the current (moving) image
inline Matrix3 fundamentalMatrix(const Transformation& T_cam0_cam1,
                                 const ze::CameraRig& cam_rig)
{
  CHECK(cam_rig.size() == 2);
  Matrix3 K0, K1;
  const ze::VectorX projection_parameteres0 = cam_rig.at(0).projectionParameters();
  const ze::VectorX projection_parameteres1 = cam_rig.at(1).projectionParameters();

  K0 << projection_parameteres0(0), 0, projection_parameteres0(2),
      0, projection_parameteres0(1), projection_parameteres0(3),
      0, 0, 1;
  K1 << projection_parameteres1(0), 0, projection_parameteres1(2),
      0, projection_parameteres1(1), projection_parameteres1(3),
      0, 0, 1;

  return (K0.inverse().transpose() * essentialMatrix(T_cam0_cam1) * K1.inverse().transpose());
}

// ----------------------------------------------------------------------------
//! Compute fundamental matrix from given camera transformation
//! @note Function assumes a camera rig with two cameras and cam0 recording the
//!       reference image and cam1 the current (moving) image
inline Matrix3 fundamentalMatrix(const Transformation& T_ref_cur,
                                 const ze::Camera& cam)
{
  Matrix3 K;
  const ze::VectorX projection_parameteres = cam.projectionParameters();

  K << projection_parameteres(0), 0, projection_parameteres(2),
      0, projection_parameteres(1), projection_parameteres(3),
      0, 0, 1;

  return (K.inverse().transpose() * essentialMatrix(T_ref_cur) * K.inverse().transpose());
}

} // namespace ze
