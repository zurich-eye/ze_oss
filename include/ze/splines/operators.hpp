// Copyright (c) 2011-2013, Paul Furgale and others.
// All rights reserved.
//
// Adopted from https://github.com/ethz-asl/Schweizer-Messer/ (2016)
// BSD Licensed

#pragma once

#include <vector>
#include <Eigen/Core>
#include <ze/common/types.h>
#include <ze/common/matrix.h>

namespace ze {
// imported from Schweizer-Messer Kinematics. Not intended to be used outside
// of the spline package.
namespace sm {

inline Matrix4 boxPlus(const Vector6 & dt)
{
  Matrix4 Bp;
  Bp <<   0.0,  -dt[5],   dt[4],  -dt[0],
          dt[5],     0.0,  -dt[3],  -dt[1],
         -dt[4],   dt[3],     0.0,  -dt[2],
          0.0,     0.0,     0.0,     0.0;
  return Bp;
}

inline Matrix46 boxMinus(const Vector4 & p)
{
  Matrix46 Bm;
  Bm <<    p[3],     0.0,     0.0,     0.0,     -p[2],     p[1],
           0.0,    p[3],     0.0,    p[2],       0.0,    -p[0],
           0.0,     0.0,    p[3],   -p[1],      p[0],      0.0,
           0.0,     0.0,     0.0,     0.0,       0.0,      0.0;
  return Bm;
}

inline Matrix6 boxTimes(Matrix4 const & T_ba)
{
  Matrix6 Tbp = Matrix6::Zero();

  Tbp.topLeftCorner<3, 3>() = T_ba.topLeftCorner<3, 3>();
  Tbp.bottomRightCorner<3, 3>() = T_ba.topLeftCorner<3, 3>();
  Tbp.topRightCorner<3, 3>() = -skewSymmetric(T_ba(0, 3), T_ba(1, 3), T_ba(2, 3))
                               * T_ba.topLeftCorner<3, 3>();

  return Tbp;
}

} // namespace kinematics
} // namespace ze
