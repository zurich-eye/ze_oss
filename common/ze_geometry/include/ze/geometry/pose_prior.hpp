// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/types.hpp>
#include <ze/common/transformation.hpp>

namespace ze {

//! A heuristic to add a pose prior to a 6x6 Hessian and gradient vector block.
inline void applyPosePrior(
    const Transformation& T_estimate,
    const Transformation& T_prior,
    const real_t prior_weight_rot,
    const real_t prior_weight_pos,
    Eigen::Ref<Matrix6> H,
    Eigen::Ref<Vector6> g)
{
  if (prior_weight_rot > 0)
  {
    real_t H_max_diag = 0;
    for(int i = 3; i < 6; ++i)
    {
      H_max_diag = std::max(H_max_diag, std::abs(H(i,i)));
    }

    // Residual of prior:
    Vector3 r = (T_prior.getRotation().inverse() * T_estimate.getRotation()).log();

    // Jacobian w.r.t. prior:
    Matrix3 J = logmapDerivativeSO3(r);
    real_t weight = H_max_diag * prior_weight_rot;

    // Add to normal equations:
    H.bottomRightCorner<3,3>().noalias() += J.transpose() * J * weight;
    g.tail<3>().noalias() -= J.transpose() * r * weight;

    VLOG(500) << "Rot. prior Hessian = J^T * J = \n" << J.transpose() * J;
  }

  // Position prior.
  if (prior_weight_pos > 0)
  {
    real_t H_max_diag = 0;
    for(int i = 0; i < 3; ++i)
    {
      H_max_diag = std::max(H_max_diag, std::abs(H(i,i)));
    }

    // Residual of prior:
    Vector3 r = T_estimate.getPosition() - T_prior.getPosition();

    // Jacobian w.r.t. prior:
    Matrix3 J = T_estimate.getRotationMatrix();
    real_t weight = H_max_diag * prior_weight_pos;

    // Add to normal equations:
    H.topLeftCorner<3,3>().noalias() += I_3x3 * weight; // J^T * J = I_3x3
    g.head<3>().noalias() -= J.transpose() * r * weight;

    VLOG(500) << "Pos. prior Hessian = J^T * J = \n" << J.transpose() * J;
  }
}

} // namespace ze
