#pragma once

#include <functional>
#include <Eigen/Dense>
#include <ze/common/manifold.h>

namespace ze {

// The traits used for this functions are defined in common/manifold.h
template<class Y, class X>
typename Eigen::Matrix<double, traits<Y>::dimension, traits<X>::dimension>
numericalDerivative(std::function<Y(const X&)> h, const X& x, double delta = 1e-5)
{
  static constexpr int N = traits<X>::dimension;
  typedef typename Eigen::Matrix<double, traits<Y>::dimension, traits<X>::dimension> Jacobian;
  typedef typename traits<Y>::TangentVector TangentY;
  typedef typename traits<X>::TangentVector TangentX;

  // Get value at x.
  Y hx = h(x);

  // Bit of a hack for now to find number of rows.
  TangentY zeroY = traits<Y>::Local(hx, hx);
  size_t m = zeroY.size();

  // Prepare a tangent vector to perturb x.
  TangentX dx;
  dx.setZero();

  // Compute numerical Jacobian column by column.
  Jacobian H;
  H.setZero();
  double factor = 1.0 / (2.0 * delta);
  for(int i = 0; i < N; ++i)
  {
    dx(i) = delta;
    TangentY dy1 = traits<Y>::Local(hx, h(traits<X>::Retract(x, dx)));
    dx(i) = -delta;
    TangentY dy2 = traits<Y>::Local(hx, h(traits<X>::Retract(x, dx)));
    dx(i) = 0;
    H.col(i) << (dy1 - dy2) * factor;
  }
  return H;
}

} // namespace ze
