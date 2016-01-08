#pragma once

#include <vector>
#include <Eigen/Core>

namespace ze {

//! Transform Eigen::Vector to std::vector.
template <typename DerivedVec>
std::vector<typename DerivedVec::Scalar> eigenVectorToStlVector(
    const Eigen::MatrixBase<DerivedVec>& ev)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY()
  std::vector<typename DerivedVec::Scalar> rv(ev.size());
  for(int i = 0; i < ev.size(); ++i)
  {
    rv[i] = ev(i);
  }
  return rv;
}

} // namespace ze
