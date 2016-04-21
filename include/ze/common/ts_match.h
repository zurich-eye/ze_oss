#pragma once

#include <cstdint>
#include <Eigen/Core>

#include <ze/common/time_conversions.h>
#include <ze/common/buffer.h>

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
