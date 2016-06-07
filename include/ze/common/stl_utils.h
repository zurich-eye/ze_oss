#pragma once

#include <vector>
#include <ze/common/types.h>
#include <ze/common/logging.hpp>

namespace ze {

// -----------------------------------------------------------------------------
//! Transform Eigen::Vector to std::vector.
template <typename DerivedVec>
std::vector<typename DerivedVec::Scalar> eigenVectorToStlVector(
    const Eigen::MatrixBase<DerivedVec>& v)
{
  //! @todo: both data is continuous, can we do this more efficiently?
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  std::vector<typename DerivedVec::Scalar> rv(v.size());
  for(int i = 0; i < v.size(); ++i)
  {
    rv[i] = v(i);
  }
  return rv;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form 0 to stop.
inline std::vector<uint32_t> range(uint32_t stop)
{
  std::vector<uint32_t> vec(stop);
  for (uint32_t i = 0u; i < stop; ++i)
  {
    vec[i] = i;
  }
  return vec;
}

// -----------------------------------------------------------------------------
//! @return Returns a vector of indices form start to stop.
inline std::vector<uint32_t> range(uint32_t start, uint32_t stop)
{
  DEBUG_CHECK_GE(stop, start);
  std::vector<uint32_t> vec(stop - start);
  for (uint32_t j = start, k = 0u; j < stop; ++j, ++k)
  {
    vec[k] = j;
  }
  return vec;
}

} // namespace ze
