// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <tuple>

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! Return depth in reference frame.
inline std::pair<real_t, bool> depthFromTriangulation(
    const Transformation& T_cur_ref,
    const Eigen::Ref<const Bearing>& f_ref,
    const Eigen::Ref<const Bearing>& f_cur)
{
  Matrix32 A;
  A << T_cur_ref.getRotation().rotate(f_ref), f_cur;
  Matrix2 AtA = A.transpose() * A;
  if(AtA.determinant() < 0.000001)
  {
    return std::make_pair(0.0, false);
  }
  Vector2 depths = - AtA.inverse() * A.transpose() * T_cur_ref.getPosition();
  return std::make_pair(std::abs(depths(0)), true);
}

//! Compute the position of a 3D point seen from two viewpoints. Fast
//! non-linear approximation (closed-form), as in Davide's book.
//! The resulting point is expressed in coordinate frame A.
Position triangulateNonLinear(
    const Transformation& T_A_B,
    const Eigen::Ref<const Bearing>& f_A,
    const Eigen::Ref<const Bearing>& f_B);

//! Triangulate multiple 3d points using triangulateNonLinear and compute
//! the corresponding reprojection errors.
void triangulateManyAndComputeAngularErrors(
    const Transformation& T_A_B,
    const Bearings& f_A_vec,
    const Bearings& f_B_vec,
    Positions& p_A,
    VectorX& reprojection_erors);

//! DLT triangulation [Hartley and Zisserman, 2nd edition, p. 312].
//! @param T_C_W vector of camera poses (camera in world coordinates).
//! @param f_C bearing vectors in camera frame.
//! @param rank_tol SVD rank tolerance.
//! @return Triangulated point, in homogeneous coordinates.
//! @return Success.
std::pair<Vector4, bool> triangulateHomogeneousDLT(
    const TransformationVector& T_C_W,
    const Bearings& p_C,
    const real_t rank_tol = 1e-9);

//! Non-linear least squares refinement of the triangulation using Gauss-Newton.
void triangulateGaussNewton(
    const TransformationVector& T_C_W,
    const Bearings& p_C,
    Eigen::Ref<Position> p_W);

} // namespace ze
