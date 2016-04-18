#pragma once

#include <utility>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <ze/common/logging.hpp>

#include <ze/common/stl_utils.h>

namespace ze {

template <typename DerivedVec>
std::pair<typename DerivedVec::Scalar, bool> median(const Eigen::MatrixBase<DerivedVec>& v)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  if(v.size() == 0)
  {
    LOG(WARNING) << "Median computation of empty vector.";
    return std::make_pair(0.0, false);
  }
  std::vector<typename DerivedVec::Scalar> w = eigenVectorToStlVector(v);
  const size_t center = w.size() / 2;
  std::nth_element(w.begin(), w.begin() + center, w.end());
  typename DerivedVec::Scalar median = w[center];
  return std::make_pair(median, true);
}

} // namespace ze
