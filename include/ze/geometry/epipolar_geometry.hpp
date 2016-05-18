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
inline Matrix3 fundamentalMatrix(const Transformation& T_ref_cur,
                                 const VectorX& proj_params_ref,
                                 const VectorX& proj_params_cur)
{
  CHECK_GE(proj_params_ref.size(), 4u);
  CHECK_GE(proj_params_cur.size(), 4u);

  Matrix3 K_ref, K_cur;
  K_ref << proj_params_ref(0), 0, proj_params_ref(2),
           0, proj_params_ref(1), proj_params_ref(3),
           0, 0, 1;
  K_cur << proj_params_cur(0), 0, proj_params_cur(2),
           0, proj_params_cur(1), proj_params_cur(3),
           0, 0, 1;

  return (K_ref.inverse().transpose() * essentialMatrix(T_ref_cur) * K_cur.inverse());
}

} // namespace ze
