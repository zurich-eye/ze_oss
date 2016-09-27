// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2011-2013, Paul Furgale and others.
// All rights reserved.
//
// Unlike otherwise stated in source files, the code in this repository is
// published under the Revised BSD (New BSD) license.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the <organization> nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once

#include <vector>
#include <Eigen/Core>
#include <ze/common/types.hpp>
#include <ze/common/matrix.hpp>

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
