#pragma once

#include <vector>
#include <Eigen/Core>

namespace ze {

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

} // namespace ze
