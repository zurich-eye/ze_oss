#pragma once

#include <glog/logging.h>

#include <ze/common/manifold.h>
#include <ze/common/numerical_derivative.h>

namespace ze {

template<typename T>
void testManifoldInvariants(const T& a, const T& b, double tol = 1e-9)
{
  CHECK(traits<T>::Equals(a, a));
  typename traits<T>::TangentVector v = traits<T>::Local(a, b);
  T c = traits<T>::Retract(a, v);
  CHECK(traits<T>::Equals(b, c, tol));
}


template<typename T>
void testRetractJacobians(const T& a, const T& b, double tol = 1e-9)
{
  using namespace std::placeholders; // for _1, _2
  typename traits<T>::Jacobian H1, H2;
  typename traits<T>::TangentVector v = traits<T>::Local(a, b);
  T c = traits<T>::Retract(a, v, &H1, &H2);
  typename traits<T>::Jacobian H1_numerical =
      numericalDerivative<T, T>(
        std::bind(traits<T>::Retract, _1, v, nullptr, nullptr), a);
  CHECK(traits<typename traits<T>::Jacobian>::Equals(H1, H1_numerical, tol));

  // TODO(cfo): Check second argument
}

} // namespace ze
