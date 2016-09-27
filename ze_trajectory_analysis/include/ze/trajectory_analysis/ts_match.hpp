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

#pragma once

#include <cstdint>
#include <Eigen/Core>

#include <ze/common/time_conversions.hpp>
#include <ze/common/buffer.hpp>

namespace ze {

template<typename BuffScalar, int BuffDim>
bool findNearestTimeStamp(Buffer<BuffScalar, BuffDim>& in_buff,
                          const int64_t& in_ts,
                          int64_t& out_ts,
                          Eigen::Matrix<BuffScalar, BuffDim, 1>& out_data,
                          double max_diff_secs=0.02,
                          double offset_secs=0.0)
{
  bool success;
  int64_t offset_nsecs = ze::secToNanosec(offset_secs);
  std::tie(out_ts, out_data, success) = in_buff.getNearestValue(in_ts+offset_nsecs);
  if(!success)
  {
    return false;
  }
  int64_t max_diff_nsecs = ze::secToNanosec(max_diff_secs);
  if(std::abs(out_ts - in_ts) > max_diff_nsecs)
  {
    return false;
  }
  else return true;
}

} // ze namespace
