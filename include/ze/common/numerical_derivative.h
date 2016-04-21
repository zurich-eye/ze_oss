#pragma once

#include <iostream>
#include <functional>
#include <ze/common/types.h>
#include <ze/common/manifold.h>

namespace ze {

// The traits used for this functions are defined in common/manifold.h
template<class Y, class X>
typename Eigen::Matrix<FloatType, traits<Y>::dimension, traits<X>::dimension>
numericalDerivative(std::function<Y(const X&)> h, const X& x, FloatType delta = 1e-5)
{
  static constexpr int N = traits<X>::dimension;
  typedef typename Eigen::Matrix<FloatType, traits<Y>::dimension, traits<X>::dimension> Jacobian;
  typedef typename traits<Y>::TangentVector TangentY;
  typedef typename traits<X>::TangentVector TangentX;

  // Get value at x.
  Y hx = h(x);

  // Prepare a tangent vector to perturb x.
  TangentX dx = TangentX::Zero();

  // Compute numerical Jacobian column by column.
  Jacobian H = Jacobian::Zero();
  FloatType factor = 1.0 / (2.0 * delta);
  for(int i = 0; i < N; ++i)
  {
    dx(i) = delta;
    TangentY dy1 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = -delta;
    TangentY dy2 = traits<Y>::local(hx, h(traits<X>::retract(x, dx)));
    dx(i) = 0;
    H.col(i) << (dy1 - dy2) * factor;
  }
  return H;
}

} // namespace ze
