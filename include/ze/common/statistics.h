#pragma once

#include <utility>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>
#include <ze/common/logging.hpp>

#include <ze/common/stl_utils.h>

namespace ze {

//! Does not take const-ref because vector will be sorted.
template <typename Scalar>
std::pair<Scalar, bool> median(std::vector<Scalar>& v)
{
  if(v.size() == 0)
  {
    LOG(WARNING) << "Median computation of empty vector.";
    return std::make_pair(Scalar{0}, false);
  }
  const size_t center = v.size() / 2;
  std::nth_element(v.begin(), v.begin() + center, v.end());
  Scalar median = v[center];
  return std::make_pair(median, true);
}

template <typename DerivedVec>
std::pair<typename DerivedVec::Scalar, bool> median(const Eigen::MatrixBase<DerivedVec>& v)
{
  EIGEN_STATIC_ASSERT_VECTOR_ONLY(DerivedVec);
  auto w = eigenVectorToStlVector(v);
  return median<typename DerivedVec::Scalar>(w);
}

template<class T>
inline T normPdf(const T x, const T mean, const T sigma)
{
  T exponent = x - mean;
  exponent *= -exponent;
  exponent /= 2 * sigma * sigma;
  T result = std::exp(exponent);
  result /= sigma * std::sqrt(2 * M_PI);
  return result;
}

} // namespace ze
