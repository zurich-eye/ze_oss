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
inline Matrix3 fundamentalMatrix(const Transformation& T_cam0_cam1,
                                 const ze::VectorX& projection_parameteres0,
                                 const ze::VectorX& projection_parameteres1)
{
  CHECK(projection_parameteres0.size() >= 4);
  CHECK(projection_parameteres1.size() >= 4);

  Matrix3 K0, K1;
  K0 << projection_parameteres0(0), 0, projection_parameteres0(2),
      0, projection_parameteres0(1), projection_parameteres0(3),
      0, 0, 1;
  K1 << projection_parameteres1(0), 0, projection_parameteres1(2),
      0, projection_parameteres1(1), projection_parameteres1(3),
      0, 0, 1;

  return (K0.inverse().transpose() * essentialMatrix(T_cam0_cam1) * K1.inverse());
}

} // namespace ze
