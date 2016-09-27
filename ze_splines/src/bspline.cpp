// Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland
// Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland
// Copyright (c) 2016, Luc Oth
// Copyright (C) 2016 ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
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
//
// Derived from https://github.com/ethz-asl/kalibr/ (2016)

#include <ze/splines/bspline.hpp>
#include <Eigen/Cholesky>
#include <Eigen/LU>
#include <Eigen/QR>
#include <boost/tuple/tuple.hpp>

namespace ze {

BSpline::BSpline(int spline_order)
  : spline_order_(spline_order)
{
  CHECK_GE(spline_order_, 2)
      << "The B-spline order must be greater than or equal to 2";
}

BSpline::~BSpline()
{
}

int BSpline::spline_order() const
{
  return spline_order_;
}

int BSpline::dimension() const
{
  return coefficients_.rows();
}

int BSpline::polynomialDegree() const
{
  return spline_order_ - 1;
}

void BSpline::setKnotsAndCoefficients(const std::vector<real_t>& knots,
                                      const MatrixX& coefficients)
{
  //std::cout << "setting " << knots.size() << " knots\n";
  // This will throw an exception if it is an invalid knot sequence.
  verifyKnotSequence(knots);

  // Check if the number of coefficients matches the number of knots.
  CHECK_EQ(numCoefficientsRequired(numValidTimeSegments(knots.size())), coefficients.cols())
       <<  "A B-spline of order " << spline_order_ << " requires "
       << numCoefficientsRequired(numValidTimeSegments(knots.size()))
       << " coefficients for the " << numValidTimeSegments(knots.size())
       << " time segments defined by " << knots.size() << " knots";

  knots_ = knots;
  coefficients_ = coefficients;

  initializeBasisMatrices();
}

void BSpline::initializeBasisMatrices()
{
  basis_matrices_.resize(numValidTimeSegments());

  for(unsigned i = 0; i < basis_matrices_.size(); i++)
  {
    basis_matrices_[i] = M(spline_order_,i + spline_order_ - 1);
  }
}

MatrixX BSpline::M(int k, int i)
{
  CHECK_GE(k, 1) << "The parameter k must be greater than or equal to 1";
  // \todo: redo these checks.
  CHECK_GE(i, 0) << "The parameter i must be greater than or equal to 0";
  CHECK_LT(i, (int)knots_.size())
      << "The parameter i must be less than the number of time segments";
  if(k == 1)
  {
    // The base-case for recursion.
    MatrixX M(1,1);
    M(0,0) = 1;
    return M;
  }
  else
  {
    MatrixX M_km1 = M(k-1,i);
    // The recursive equation for M
    // M_k = [ M_km1 ] A  + [  0^T  ] B
    //       [  0^T  ]      [ M_km1 ]
    //        -------        -------
    //         =: M1          =: M2
    //
    //     = M1 A + M2 B
    MatrixX M1 = MatrixX::Zero(M_km1.rows() + 1, M_km1.cols());
    MatrixX M2 = MatrixX::Zero(M_km1.rows() + 1, M_km1.cols());

    M1.topRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;
    M2.bottomRightCorner(M_km1.rows(),M_km1.cols()) = M_km1;

    MatrixX A = MatrixX::Zero(k-1, k);
    for(int idx = 0; idx < A.rows(); idx++)
    {
      int j = i - k + 2 + idx;
      real_t d0 = d_0(k, i, j);
      A(idx, idx  ) = 1.0 - d0;
      A(idx, idx+1) = d0;
    }

    MatrixX B = MatrixX::Zero(k-1, k);
    for(int idx = 0; idx < B.rows(); idx++)
    {
      int j = i - k + 2 + idx;
      real_t d1 = d_1(k, i, j);
      B(idx, idx  ) = -d1;
      B(idx, idx+1) = d1;
    }

    MatrixX M_k;

    return M_k = M1 * A + M2 * B;
  }
}

real_t BSpline::d_0(int k, int i, int j)
{
  CHECK_LE(j+k-1.0, (int)knots_.size()) <<  "Index out of range with k=" << k
                                        << ", i=" << i << ", and j=" << j;
  CHECK_LT(0, (int)knots_.size()) <<  "Index out of range with k=" << k
                                  << ", i=" << i << ", and j=" << j;
  CHECK_LE(j, (int)knots_.size()) <<  "Index out of range with k=" << k
                                  << ", i=" << i << ", and j=" << j;
  CHECK_LE(i, (int)knots_.size()) <<  "Index out of range with k=" << k
                                  << ", i=" << i << ", and j=" << j;

  real_t denom = knots_[j+k-1] - knots_[j];
  if(denom <= 0.0)
  {
    return 0.0;
  }

  real_t numerator = knots_[i] - knots_[j];

  return numerator/denom;
}

real_t BSpline::d_1(int k, int i, int j)
{
  CHECK_LE(j+k-1.0, (int)knots_.size()) <<  "Index out of range with k="
                                        << k << ", i=" << i << ", and j=" << j;
  CHECK_LT(0, (int)knots_.size()) <<  "Index out of range with k="
                                  << k << ", i=" << i << ", and j=" << j;
  CHECK_LE(j, (int)knots_.size()) <<  "Index out of range with k="
                                  << k << ", i=" << i << ", and j=" << j;
  CHECK_LE(i, (int)knots_.size()) <<  "Index out of range with k="
                                  << k << ", i=" << i << ", and j=" << j;
  real_t denom = knots_[j+k-1] - knots_[j];
  if(denom <= 0.0)
  {
    return 0.0;
  }

  real_t numerator = knots_[i+1] - knots_[i];

  return numerator/denom;
}

void BSpline::setKnotVectorAndCoefficients(const VectorX& knots,
                                           const MatrixX& coefficients)
{
  std::vector<real_t> k(knots.size());
  for(unsigned i = 0; i < k.size(); i++)
  {
    k[i] = knots(i);
  }

  setKnotsAndCoefficients(k, coefficients);
}

const std::vector<real_t> BSpline::knots() const
{
  return knots_;
}

VectorX BSpline::knotVector() const
{
  VectorX k(knots_.size());
  for(unsigned i = 0; i < knots_.size(); i++)
  {
    k(i) = knots_[i];
  }

  return k;
}

const MatrixX& BSpline::coefficients() const
{
  return coefficients_;
}

void BSpline::verifyKnotSequence(const std::vector<real_t>& knots)
{
  CHECK_GE((int)knots.size(), minimumKnotsRequired())
      << "The sequence does not contain enough knots to define an active time sequence "
      << "for a B-spline of order " << spline_order_
      << ". At least " << minimumKnotsRequired()
      << " knots are required";

  for(unsigned i = 1; i < knots_.size(); i++)
  {
    CHECK_LE(knots[i-1], knots[i])
        << "The knot sequence must be nondecreasing. Knot " << i
        << " was not greater than or equal to knot " << (i-1);
  }
}

int BSpline::numValidTimeSegments(int numKnots) const
{
  int nv = numKnots - 2*spline_order_ + 1;
  return std::max(nv,0);
}

int BSpline::numValidTimeSegments() const
{
  return numValidTimeSegments(knots_.size());
}

int BSpline::minimumKnotsRequired() const
{
  return numKnotsRequired(1);
}

int BSpline::numCoefficientsRequired(int num_time_segments) const
{
  return num_time_segments + spline_order_ - 1;
}

int BSpline::numKnotsRequired(int num_time_segments) const
{
  return numCoefficientsRequired(num_time_segments) + spline_order_;
}

real_t BSpline::t_min() const
{
  CHECK_GE((int)knots_.size(), minimumKnotsRequired())
      << "The B-spline is not well initialized";
  return knots_[spline_order_ - 1];
}

real_t BSpline::t_max() const
{
  CHECK_GE((int)knots_.size(), minimumKnotsRequired())
      << "The B-spline is not well initialized";
  return knots_[knots_.size() - spline_order_];
}

std::pair<real_t,int> BSpline::computeTIndex(real_t t) const
{
  CHECK_GE(t, t_min()) << "The time is out of range by " << (t - t_min());

  //// HACK - avoids numerical problems on initialisation
  if (std::abs(t_max() - t) < 1e-10)
  {
    t = t_max();
  }
  //// \HACK

  CHECK_LE(t, t_max())
      << "The time is out of range by " << (t_max() - t);
  std::vector<real_t>::const_iterator i;
  if(t == t_max())
  {
    // This is a special case to allow us to evaluate the spline at the boundary of the
    // interval. This is not stricly correct but it will be useful when we start doing
    // estimation and defining knots at our measurement times.
    i = knots_.end() - spline_order_;
  }
  else
  {
    i = std::upper_bound(knots_.begin(), knots_.end(), t);
  }
  //CHECK_NE(i, knots_.end()) << "Something very bad has happened in computeTIndex(" << t << ")";

  // Returns the index of the knot segment this time lies on and the width of this knot segment.
  return std::make_pair(*i - *(i-1),(i - knots_.begin()) - 1);

}

std::pair<real_t,int> BSpline::computeUAndTIndex(real_t t) const
{
  std::pair<real_t,int> ui = computeTIndex(t);

  int index = ui.second;
  real_t denom = ui.first;

  if(denom <= 0.0)
  {
    // The case of duplicate knots.
    //std::cout << "Duplicate knots\n";
    return std::make_pair(0, index);
  }
  else
  {
    real_t u = (t - knots_[index])/denom;

    return std::make_pair(u, index);
  }
}

int dmul(int i, int derivative_order)
{
  if(derivative_order == 0)
  {
    return 1;
  }
  else if(derivative_order == 1)
  {
    return i;
  }
  else
  {
    return i * dmul(i-1,derivative_order-1) ;
  }
}

VectorX BSpline::computeU(real_t uval,
                          int segmentIndex,
                          int derivativeOrder) const
{
  VectorX u = VectorX::Zero(spline_order_);
  real_t delta_t = knots_[segmentIndex+1] - knots_[segmentIndex];
  real_t multiplier = 0.0;
  if(delta_t > 0.0)
  {
    multiplier = 1.0/pow(delta_t, derivativeOrder);
  }

  real_t uu = 1.0;
  for(int i = derivativeOrder; i < spline_order_; i++)
  {
    u(i) = multiplier * uu * dmul(i,derivativeOrder) ;
    uu = uu * uval;
  }

  return u;
}

VectorX BSpline::eval(real_t t) const
{
  return evalD(t,0);
}

const MatrixX& BSpline::basisMatrixFromKnotIndex(int knot_index) const
{
  return basis_matrices_[basisMatrixIndexFromStartingKnotIndex(knot_index)];
}

VectorX BSpline::evalD(real_t t, int derivative_order) const
{
  CHECK_GE(derivative_order, 0) << "To integrate, use the integral function";
  // Returns the normalized u value and the lower-bound time index.
  std::pair<real_t,int> ui = computeUAndTIndex(t);
  VectorX u = computeU(ui.first, ui.second, derivative_order);

  int bidx = ui.second - spline_order_ + 1;

  // Evaluate the spline (or derivative) in matrix form.
  //
  // [c_0 c_1 c_2 c_3] * B^T * u
  // spline coefficients

  VectorX rv = coefficients_.block(0,bidx,coefficients_.rows(),spline_order_)
               * basis_matrices_[bidx].transpose() * u;

  return rv;
}

VectorX BSpline::evalDAndJacobian(real_t t,
                                  int derivative_order,
                                  MatrixX* Jacobian,
                                  VectorXi* coefficient_indices) const
{
  CHECK_GE(derivative_order, 0) << "To integrate, use the integral function";
  // Returns the normalized u value and the lower-bound time index.
  std::pair<real_t,int> ui = computeUAndTIndex(t);
  VectorX u = computeU(ui.first, ui.second, derivative_order);

  int bidx = ui.second - spline_order_ + 1;

  // Evaluate the spline (or derivative) in matrix form.
  //
  // [c_0 c_1 c_2 c_3] * B^T * u
  // spline coefficients

  // The spline value
  VectorX Bt_u = basis_matrices_[bidx].transpose() * u;
  VectorX v = coefficients_.block(0,bidx,coefficients_.rows(),spline_order_) * Bt_u;

  if(Jacobian)
  {
    // The Jacobian
    Jacobian->resize(coefficients_.rows(), Bt_u.size() * coefficients_.rows());
    MatrixX one = MatrixX::Identity(coefficients_.rows(), coefficients_.rows());
    for(int i = 0; i < Bt_u.size(); i++)
    {
      Jacobian->block(0, i*coefficients_.rows(),
                      coefficients_.rows(),
                      coefficients_.rows()) = one * Bt_u[i];
    }
  }

  if(coefficient_indices)
  {
    int D = coefficients_.rows();
    *coefficient_indices = VectorXi::LinSpaced(spline_order_ * D,
                                              bidx * D,
                                              (bidx + spline_order_) * D - 1);
  }

  return v;
}

std::pair<VectorX, MatrixX> BSpline::evalDAndJacobian(real_t t,
                                                      int derivative_order) const
{
  std::pair<VectorX, MatrixX> rv;

  rv.first = evalDAndJacobian(t, derivative_order, &rv.second, NULL);

  return rv;
}

MatrixX BSpline::localBasisMatrix(real_t t, int derivative_order) const
{
  return Phi(t,derivative_order);
}

MatrixX BSpline::localCoefficientMatrix(real_t t) const
{
  std::pair<real_t,int> ui = computeTIndex(t);
  int bidx = ui.second - spline_order_ + 1;

  return coefficients_.block(0,bidx,coefficients_.rows(),spline_order_);
}

VectorX BSpline::localCoefficientVector(real_t t) const
{

  std::pair<real_t,int> ui = computeTIndex(t);
  int bidx = ui.second - spline_order_ + 1;
  VectorX c(spline_order_ * coefficients_.rows());
  for(int i = 0; i < spline_order_; i++)
  {
    c.segment(i*coefficients_.rows(), coefficients_.rows()) = coefficients_.col(i + bidx);
  }

  return c;
}

VectorXi BSpline::localCoefficientVectorIndices(real_t t) const
{
  std::pair<real_t,int> ui = computeTIndex(t);
  int bidx = ui.second - spline_order_ + 1;
  int D = coefficients_.rows();

  return VectorXi::LinSpaced(spline_order_*D,bidx*D,(bidx + spline_order_)*D - 1);
}

VectorXi BSpline::localVvCoefficientVectorIndices(real_t t) const
{
  std::pair<real_t,int> ui = computeTIndex(t);
  int bidx = ui.second - spline_order_ + 1;

  return VectorXi::LinSpaced(spline_order_,bidx,(bidx + spline_order_) - 1);
}

MatrixX BSpline::Phi(real_t t, int derivative_order) const
{
  CHECK_GE(derivative_order, 0) << "To integrate, use the integral function";
  std::pair<real_t,int> ui = computeUAndTIndex(t);

  VectorX u = computeU(ui.first, ui.second, derivative_order);

  int bidx = ui.second - spline_order_ + 1;

  u = basis_matrices_[bidx].transpose() * u;

  MatrixX Phi = MatrixX::Zero(coefficients_.rows(),
                              spline_order_*coefficients_.rows());
  MatrixX one = MatrixX::Identity(Phi.rows(), Phi.rows());
  for(int i = 0; i < spline_order_; i++)
  {
    Phi.block(0,Phi.rows()*i,Phi.rows(),Phi.rows()) = one * u(i);
  }

  return Phi;
}

void BSpline::setCoefficientVector(const VectorX& c)
{
  CHECK_GE(c.size(), coefficients_.rows() * coefficients_.cols())
      << "The coefficient vector is the wrong size. The vector must contain all vector-valued coefficients stacked up into one column.";
  for(int i = 0; i < coefficients_.cols(); i++)
  {
    coefficients_.col(i) = c.segment(i * coefficients_.rows(),coefficients_.rows());
  }
}

VectorX BSpline::coefficientVector()
{
  VectorX c(coefficients_.rows() * coefficients_.cols());
  for(int i = 0; i < coefficients_.cols(); i++)
  {
    c.segment(i * coefficients_.rows(),coefficients_.rows()) = coefficients_.col(i);
  }
  return c;
}

void BSpline::setCoefficientMatrix(const MatrixX& coefficients)
{
  CHECK_EQ(coefficients_.rows(), coefficients.rows())
      << "The new coefficient matrix must match the size of the existing coefficient matrix";
  CHECK_EQ(coefficients_.cols(), coefficients.cols())
      << "The new coefficient matrix must match the size of the existing coefficient matrix";
  coefficients_ = coefficients;
}

const MatrixX& BSpline::basisMatrix(int i) const
{
  CHECK_LE(i, numValidTimeSegments()) << "index out of range";
  CHECK_LE(0, numValidTimeSegments()) << "index out of range";
  return basis_matrices_[i];
}


std::pair<real_t,real_t> BSpline::timeInterval() const
{
  return std::make_pair(t_min(), t_max());
}

std::pair<real_t,real_t> BSpline::timeInterval(int i) const
{
  CHECK_GE((int)knots_.size(), minimumKnotsRequired()) << "The B-spline is not well initialized";
  CHECK_LE(i, numValidTimeSegments()) << "index out of range";
  CHECK_LT(0, numValidTimeSegments()) << "index out of range";
  return std::make_pair(knots_[spline_order_ + i - 1],knots_[spline_order_ + i]);
}

void BSpline::initSpline(real_t t_0, real_t t_1,
                         const VectorX& p_0,
                         const VectorX& p_1)
{
  CHECK_EQ(p_0.size(), p_1.size())
      << "The coefficient vectors should be the same size";
  CHECK_GT(t_1, t_0) << "Time must be increasing from t_0 to t_1";

  // Initialize the spline so that it interpolates the two points
  // and moves between them with a constant velocity.

  // How many knots are required for one time segment?
  int K = numKnotsRequired(1);
  // How many coefficients are required for one time segment?
  int C = numCoefficientsRequired(1);
  // What is the vector coefficient dimension
  int D = p_0.size();

  // Initialize a uniform knot sequence
  real_t dt = t_1 - t_0;
  std::vector<real_t> knots(K);
  for(int i = 0; i < K; i++)
  {
    knots[i] = t_0 + (i - spline_order_ + 1) * dt;
  }
  // Set the knots and zero the coefficients
  setKnotsAndCoefficients(knots, MatrixX::Zero(D,C));

  // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
  int coefficientDim = C * D;
  // We always need an even number of constraints.
  int constraintsRequired = C + (C & 0x1);
  int constraintSize = constraintsRequired * D;

  MatrixX A = MatrixX::Zero(constraintSize, coefficientDim);
  VectorX b = VectorX::Zero(constraintSize);

  // Add the position constraints.
  int brow = 0;
  int bcol = 0;
  A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),0);
  b.segment(brow,D) = p_0;
  brow += D;
  A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),0);
  b.segment(brow,D) = p_1;
  brow += D;

  if(spline_order_ > 2)
  {
    // At the very minimum we have to add velocity constraints.
    VectorX v = (p_1 - p_0)/dt;
    A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),1);
    b.segment(brow,D) = v;
    brow += D;
    A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),1);
    b.segment(brow,D) = v;
    brow += D;

    if(spline_order_ > 4)
    {
      // Now we add the constraint that all higher-order derivatives are zero.
      int derivativeOrder = 2;
      VectorX z = VectorX::Zero(D);
      while(brow < A.rows())
      {
        A.block(brow,bcol,D,coefficientDim) = Phi(t_min(),derivativeOrder);
        b.segment(brow,D) = z;
        brow += D;
        A.block(brow,bcol,D,coefficientDim) = Phi(t_max(),derivativeOrder);
        b.segment(brow,D) = z;
        brow += D;
        ++derivativeOrder;
      }
    }
  }

  // Now we solve the Ax=b system
  if(A.rows() != A.cols())
  {
    // The system is over constrained. This happens for odd ordered splines.
    b = (A.transpose() * b).eval();
    A = (A.transpose() * A).eval();
  }

  // Solve for the coefficient vector.
  VectorX c = A.householderQr().solve(b);
  // ldlt doesn't work for this problem. It may be because the ldlt decomposition
  // requires the matrix to be positive or negative semidefinite
  // http://eigen.tuxfamily.org/dox-devel/TutorialLinearAlgebra.html#TutorialLinAlgRankRevealing
  // which may imply that it is symmetric. Our A matrix is only symmetric in the over-constrained case.
  //VectorX c = A.ldlt().solve(b);
  setCoefficientVector(c);
}

void BSpline::addCurveSegment(real_t t, const VectorX& p_1)
{
  CHECK_GT(t, t_max())
      << "The new time must be past the end of the last valid segment";
  CHECK_EQ(p_1.size(), coefficients_.rows()) << "Invalid coefficient vector size";

  // Get the final valid time interval.
  int NT = numValidTimeSegments();
  std::pair<real_t, real_t> interval_km1 = timeInterval(NT-1);

  VectorX p_0;

  // Store the position of the spline at the  end of the interval.
  // We will use these as constraints as we don't want them to change.
  p_0 = eval(interval_km1.second);

  // Retool the knot vector.
  real_t du;
  int km1;
  std::tie(du,km1) = computeTIndex(interval_km1.first);

  // leave knots km1 and k alone but retool the other knots.
  real_t dt = t - knots_[km1 + 1];
  real_t kt = t;

  // add another knot.
  std::vector<real_t> knots(knots_);
  knots.push_back(0.0);
  // space the further knots uniformly.
  for(unsigned k = km1 + 2; k < knots.size(); k++)
  {
    knots[k] = kt;
    kt += dt;
  }
  // Tack on an new, uninitialized coefficient column.
  MatrixX c(coefficients_.rows(), coefficients_.cols() + 1);
  c.topLeftCorner(coefficients_.rows(), coefficients_.cols()) = coefficients_;
  setKnotsAndCoefficients(knots,c);

  // Now, regardless of the order of the spline, we should only have to add
  // a single knot and coefficient vector.
  // In this case, we should solve for the last two coefficient vectors
  // (i.e., the new one and the one before the new one).

  // Get the time interval of the new time segment.
  real_t t_0, t_1;
  std::tie(t_0,t_1) = timeInterval(NT);

  // what is the coefficient dimension?
  int D = coefficients_.rows();
  // How many vector-valued coefficients are required? In this case, 2.
  // We will leave the others fixed.
  int C = 2;
  // Now we have to solve an Ax = b linear system to determine the
  // correct coefficient vectors.
  int coefficientDim = C * D;
  // We always need an even number of constraints.
  int constraintsRequired = 2;
  int constraintSize = constraintsRequired * D;

  MatrixX A = MatrixX::Zero(constraintSize, coefficientDim);
  VectorX b = VectorX::Zero(constraintSize);      // Build the A matrix.

  int phiBlockColumnOffset = D * std::max(0,(spline_order_ - 2));
  VectorX fixedCoefficients = localCoefficientVector(t_0).segment(0,
                                                                  phiBlockColumnOffset);

  // Add the position constraints.
  int brow = 0;
  int bcol = 0;
  MatrixX P;
  P = Phi(t_0,0);
  A.block(brow,bcol,D,coefficientDim) = P.block(0, phiBlockColumnOffset,
                                                D, coefficientDim);
  b.segment(brow,D) = p_0 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;
  brow += D;

  P = Phi(t_1,0);
  A.block(brow,bcol,D,coefficientDim) = P.block(0,phiBlockColumnOffset,
                                                D, coefficientDim);
  b.segment(brow,D) = p_1 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;;
  brow += D;

  // Add regularization constraints (keep the coefficients small)
  //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-4 * MatrixX::Identity(coefficientDim, coefficientDim);
  //b.segment(brow,coefficientDim) = VectorX::Zero(coefficientDim);
  //brow += coefficientDim;

  // Now we solve the Ax=b system
  if(A.rows() != A.cols())
  {
    // The system is over constrained. This happens for odd ordered splines.
    b = (A.transpose() * b).eval();
    A = (A.transpose() * A).eval();
  }

  VectorX cstar = A.householderQr().solve(b);
  coefficients_.col(coefficients_.cols() - 2) = cstar.head(D);
  coefficients_.col(coefficients_.cols() - 1) = cstar.tail(D);
}


void BSpline::removeCurveSegment()
{
  if(knots_.size() > 0 && coefficients_.cols() > 0)
  {
    knots_.erase(knots_.begin());
    coefficients_ = coefficients_.block(0,
                                        1,
                                        coefficients_.rows(),
                                        coefficients_.cols() - 1).eval();
  }
}

void BSpline::setLocalCoefficientVector(real_t t, const VectorX& c)
{
  CHECK_EQ(c.size(), spline_order_ * coefficients_.rows())
      << "The local coefficient vector is the wrong size";
  std::pair<real_t,int> ui = computeTIndex(t);
  int bidx = ui.second - spline_order_ + 1;
  for(int i = 0; i < spline_order_; i++)
  {
    coefficients_.col(i + bidx) = c.segment(i * coefficients_.rows(),
                                            coefficients_.rows());
  }

}

void BSpline::initSpline2(const VectorX& times,
                          const MatrixX& interpolation_points,
                          int num_segments,
                          real_t lambda)
{
  CHECK_EQ(times.size(), interpolation_points.cols())
      << "The number of times and the number of interpolation points must be equal";
  CHECK_GE(times.size(), 2) << "There must be at least two times";
  CHECK_GE(num_segments, 1) << "There must be at least one time segment";
  for(int i = 1; i < times.size(); i++)
  {
    CHECK_LE(times[i-1], times[i])
        << "The time sequence must be nondecreasing. time " << i
        << " was not greater than or equal to time " << (i-1);
  }

  // Initialize the spline so that it interpolates the N points

  // How many knots are required for one time segment?
  int K = numKnotsRequired(num_segments);
  // How many coefficients are required for one time segment?
  int C = numCoefficientsRequired(num_segments);
  // What is the vector coefficient dimension
  int D = interpolation_points.rows();

  // Initialize a uniform knot sequence
  real_t dt = (times[times.size() - 1] - times[0]) / num_segments;
  std::vector<real_t> knots(K);
  for(int i = 0; i < K; i++)
  {
    knots[i] = times[0] + (i - spline_order_ + 1) * dt;
  }
  // Set the knots and zero the coefficients
  setKnotsAndCoefficients(knots, MatrixX::Zero(D,C));

  // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
  int coefficientDim = C * D;

  int numConstraints = (knots.size() - 2 * spline_order_ + 2) + interpolation_points.cols();
  int constraintSize = numConstraints * D;

  MatrixX A = MatrixX::Zero(constraintSize, coefficientDim);
  VectorX b = VectorX::Zero(constraintSize);

  int brow = 0;
  //int bcol = 0;
  // Now add the regularization constraint.
  //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-1* MatrixX::Identity(coefficientDim, coefficientDim);
  //b.segment(brow,coefficientDim) = VectorX::Zero(coefficientDim);
  //brow += coefficientDim;
  for(int i = spline_order_ - 1; i < (int)knots.size() - spline_order_ + 1; i++)
  {
    VectorXi coeffIndices = localCoefficientVectorIndices(knots[i]);

    A.block(brow,coeffIndices[0],D,coeffIndices.size()) = lambda * Phi(knots[i],2);
    b.segment(brow,D) = VectorX::Zero(D);
    brow += D;
  }

  // Add the position constraints.
  for(int i = 0; i < interpolation_points.cols(); i++)
  {
    VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);
    A.block(brow,coeffIndices[0],D,coeffIndices.size()) = Phi(times[i],0);

    b.segment(brow,D) = interpolation_points.col(i);
    brow += D;
  }

  // Now we solve the Ax=b system
  //if(A.rows() != A.cols())
  //  {
  // The system is over constrained. This happens for odd ordered splines.
  b = (A.transpose() * b).eval();
  A = (A.transpose() * A).eval();
  //  }

  // Solve for the coefficient vector.
  VectorX c = A.ldlt().solve(b);
  // ldlt doesn't work for this problem. It may be because the ldlt decomposition
  // requires the matrix to be positive or negative semidefinite
  // http://eigen.tuxfamily.org/dox-devel/TutorialLinearAlgebra.html#TutorialLinAlgRankRevealing
  // which may imply that it is symmetric. Our A matrix is only symmetric in the over-constrained case.
  // VectorX c = A.ldlt().solve(b);
  setCoefficientVector(c);
}

void BSpline::initSpline3(const VectorX& times,
                          const MatrixX& interpolation_points,
                          int num_segments,
                          real_t lambda)
{
  CHECK_EQ(times.size(), interpolation_points.cols())
      << "The number of times and the number of interpolation points must be equal";
  CHECK_GE(times.size(), 2) << "There must be at least two times";
  CHECK_GE(num_segments, 1) << "There must be at least one time segment";
  for(int i = 1; i < times.size(); i++)
  {
    CHECK_LE(times[i-1], times[i])
        <<  "The time sequence must be nondecreasing. time " << i
        << " was not greater than or equal to time " << (i-1);
  }

  // How many knots are required for one time segment?
  int K = numKnotsRequired(num_segments);
  // How many coefficients are required for one time segment?
  int C = numCoefficientsRequired(num_segments);
  // What is the vector coefficient dimension
  int D = interpolation_points.rows();

  // Initialize a uniform knot sequence
  real_t dt = (times[times.size() - 1] - times[0]) / num_segments;
  std::vector<real_t> knots(K);
  for(int i = 0; i < K; i++)
  {
    knots[i] = times[0] + (i - spline_order_ + 1) * dt;
  }

  setKnotsAndCoefficients(knots, MatrixX::Zero(D,C));

  // Now we have to solve an Ax = b linear system to determine the correct coefficient vectors.
  int coefficientDim = C * D;

  int numConstraints = interpolation_points.cols();
  int constraintSize = numConstraints * D;

  MatrixX A = MatrixX::Zero(constraintSize, coefficientDim);
  VectorX b = VectorX::Zero(constraintSize);

  int brow = 0;
  // Add the position constraints.
  for(int i = 0; i < interpolation_points.cols(); i++)
  {
    VectorXi coeffIndices = localCoefficientVectorIndices(times[i]);

    A.block(brow,coeffIndices[0],D,coeffIndices.size()) = Phi(times[i],0);

    b.segment(brow,D) = interpolation_points.col(i);
    brow += D;
  }

  b = (A.transpose() * b).eval();
  A = (A.transpose() * A).eval();

  // Add the motion constraint.
  VectorX W = VectorX::Constant(D,lambda);

  A += curveQuadraticIntegralDiag(W, 2);

  VectorX c = A.ldlt().solve(b);
  setCoefficientVector(c);

}

void BSpline::addCurveSegment2(real_t t,
                               const VectorX& p_1,
                               real_t lambda)
{
  CHECK_GT(t, t_max())
      << "The new time must be past the end of the last valid segment";
  CHECK_EQ(p_1.size(), coefficients_.rows())
      << "Invalid coefficient vector size";

  // Get the final valid time interval.
  int NT = numValidTimeSegments();
  std::pair<real_t, real_t> interval_km1 = timeInterval(NT-1);

  VectorX p_0;

  // Store the position of the spline at the  end of the interval.
  // We will use these as constraints as we don't want them to change.
  p_0 = eval(interval_km1.second);

  // Retool the knot vector.
  real_t du;
  int km1;
  std::tie(du,km1) = computeTIndex(interval_km1.first);

  // leave knots km1 and k alone but retool the other knots.
  real_t dt = t - knots_[km1 + 1];
  real_t kt = t;

  // add another knot.
  std::vector<real_t> knots(knots_);
  knots.push_back(0.0);
  // space the further knots uniformly.
  for(unsigned k = km1 + 2; k < knots.size(); k++)
  {
    knots[k] = kt;
    kt += dt;
  }
  // Tack on an new, uninitialized coefficient column.
  MatrixX c(coefficients_.rows(), coefficients_.cols() + 1);
  c.topLeftCorner(coefficients_.rows(), coefficients_.cols()) = coefficients_;
  setKnotsAndCoefficients(knots,c);

  // Now, regardless of the order of the spline, we should only have to
  // add a single knot and coefficient vector.
  // In this case, we should solve for the last two coefficient
  // vectors (i.e., the new one and the one before the
  // new one).

  // Get the time interval of the new time segment.
  real_t t_0, t_1;
  std::tie(t_0,t_1) = timeInterval(NT);

  // what is the coefficient dimension?
  int D = coefficients_.rows();
  // How many vector-valued coefficients are required? In this case, 2.
  // We will leave the others fixed.
  int C = 2;
  // Now we have to solve an Ax = b linear system to determine the correct
  // coefficient vectors.
  int coefficientDim = C * D;
  // We always need an even number of constraints.
  int constraintsRequired = 2 + 2;
  int constraintSize = constraintsRequired * D;

  MatrixX A = MatrixX::Zero(constraintSize, coefficientDim);
  VectorX b = VectorX::Zero(constraintSize);      // Build the A matrix.

  int phiBlockColumnOffset = D * std::max(0,(spline_order_ - 2));
  VectorX fixedCoefficients = localCoefficientVector(t_0).segment(0,
                                                                  phiBlockColumnOffset);

  // Add the position constraints.
  int brow = 0;
  int bcol = 0;
  MatrixX P;
  P = Phi(t_0,0);
  A.block(brow,bcol,D,coefficientDim) = P.block(0, phiBlockColumnOffset,
                                                D, coefficientDim);
  b.segment(brow,D) = p_0 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;
  brow += D;

  P = Phi(t_1,0);
  A.block(brow,bcol,D,coefficientDim) = P.block(0, phiBlockColumnOffset,
                                                D, coefficientDim);
  b.segment(brow,D) = p_1 - P.block(0,0,D,phiBlockColumnOffset) * fixedCoefficients;;
  brow += D;


  // Add regularization constraints (keep the acceleration small)
  P = Phi(t_0,2);
  A.block(brow,bcol,D,coefficientDim) = lambda * P.block(0, phiBlockColumnOffset,
                                                         D, coefficientDim);
  b.segment(brow,D) = VectorX::Zero(D);
  brow += D;

  P = Phi(t_1,2);
  A.block(brow,bcol,D,coefficientDim) = lambda * P.block(0, phiBlockColumnOffset,
                                                         D, coefficientDim);
  b.segment(brow,D) = VectorX::Zero(D);
  brow += D;

  //A.block(brow,bcol,coefficientDim,coefficientDim) = 1e-4 * MatrixX::Identity(coefficientDim, coefficientDim);
  //b.segment(brow,coefficientDim) = VectorX::Zero(coefficientDim);
  //brow += coefficientDim;

  // Now we solve the Ax=b system
  if(A.rows() != A.cols())
  {
    // The system is over constrained. This happens for odd ordered splines.
    b = (A.transpose() * b).eval();
    A = (A.transpose() * A).eval();
  }

  VectorX cstar = A.householderQr().solve(b);
  coefficients_.col(coefficients_.cols() - 2) = cstar.head(D);
  coefficients_.col(coefficients_.cols() - 1) = cstar.tail(D);
}

MatrixX BSpline::Vi(int segment_index) const
{
  CHECK_LT(segment_index, numValidTimeSegments())
      << "Segment index out of bounds";
  CHECK_LT(0, numValidTimeSegments())
      << "Segment index out of bounds";

  VectorX vals(spline_order_*2);
  for (int i = 0; i < vals.size(); ++i)
  {
    vals[i] = 1.0/(i + 1.0);
  }

  MatrixX V(spline_order_,spline_order_);
  for(int r = 0; r < V.rows(); r++)
  {
    for(int c = 0; c < V.cols(); c++)
    {
      V(r,c) = vals[r + c];
    }
  }

  real_t t_0,t_1;
  std::tie(t_0,t_1) = timeInterval(segment_index);

  V *= t_1 - t_0;

  return V;
}

VectorX BSpline::evalIntegral(real_t t1, real_t t2) const
{
  if(t1 > t2)
  {
    return -evalIntegral(t2,t1);
  }

  std::pair<real_t,int> u1 = computeTIndex(t1);
  std::pair<real_t,int> u2 = computeTIndex(t2);

  VectorX integral = VectorX::Zero(coefficients_.rows());

  // LHS remainder.
  real_t lhs_remainder = t1 - knots_[u1.second];
  if(lhs_remainder > 1e-16 && u1.first > 1e-16)
  {
    lhs_remainder /= u1.first;
    VectorX v(spline_order_);
    real_t du = lhs_remainder;
    for(int i = 0; i < spline_order_; i++)
    {
      v(i) = du/(i + 1.0);
      du *= lhs_remainder;
    }
    int bidx = basisMatrixIndexFromStartingKnotIndex(u1.second);
    integral -= u1.first * coefficients_.block(0,
                                               bidx,coefficients_.rows(),
                                               spline_order_)
                * basis_matrices_[bidx].transpose() * v;
  }

  // central time segments.
  VectorX v = VectorX::Zero(spline_order_);
  for(int i = 0; i < spline_order_; i++)
  {
    v(i) = 1.0/(i + 1.0);
  }

  for(int s = u1.second; s < u2.second; s++)
  {
    int bidx = basisMatrixIndexFromStartingKnotIndex(s);
    integral += (knots_[s+1] - knots_[s])
        * coefficients_.block(0, bidx, coefficients_.rows(), spline_order_)
        * basis_matrices_[bidx].transpose() * v;
  }

  // RHS remainder.
  real_t rhs_remainder = t2 - knots_[u2.second];
  if(rhs_remainder > 1e-16 && u2.first > 1e-16)
  {
    rhs_remainder /= u2.first;

    VectorX v(spline_order_);
    real_t du = rhs_remainder;
    for(int i = 0; i < spline_order_; i++)
    {
      v(i) = du / (i + 1.0);
      du *= rhs_remainder;
    }

    int bidx = basisMatrixIndexFromStartingKnotIndex(u2.second);
    integral += u2.first
                * coefficients_.block(0,bidx,coefficients_.rows(),spline_order_)
                * basis_matrices_[bidx].transpose()
                * v;
  }

  return integral;
}

int BSpline::basisMatrixIndexFromStartingKnotIndex(int starting_knot_index) const
{
  return starting_knot_index - spline_order_ + 1;
}

int BSpline::startingKnotIndexFromBasisMatrixIndex(int basis_matrix_index) const
{
  return spline_order_ + basis_matrix_index - 1;
}


MatrixX BSpline::Bij(int segment_index, int column_index) const
{
  CHECK_LE(segment_index, (int)basis_matrices_.size()) << "Out of range";
  CHECK_LT(0, (int)basis_matrices_.size()) << "Out of range";
  CHECK_LE(column_index, spline_order_) << "Out of range";
  CHECK_LT(0, spline_order_) << "Out of range";
  int D = coefficients_.rows();
  MatrixX B = MatrixX::Zero(spline_order_*D,D);
  for(int i = 0; i < D; i++)
  {
    B.block(i*spline_order_,i,spline_order_,1) = basis_matrices_[segment_index].col(column_index);
  }

  return B;
}

MatrixX BSpline::Mi(int segment_index) const
{
  CHECK_LE(segment_index, (int)basis_matrices_.size()) << "Out of range";
  CHECK_LT(0, (int)basis_matrices_.size()) << "Out of range";
  int D = coefficients_.rows();
  MatrixX M = MatrixX::Zero(spline_order_*D,spline_order_*D);

  for(int j = 0; j < spline_order_; j++)
  {
    M.block(0,j*D,D*spline_order_, D) = Bij(segment_index,j);
  }

  return M;
}

VectorX BSpline::getLocalBiVector(real_t t, int derivative_order) const
{
  VectorX ret = VectorX::Zero(spline_order_);
  getLocalBiInto(t, ret, derivative_order);

  return ret;
}

void BSpline::getLocalBiInto(real_t t, VectorX& ret, int derivative_order) const
{
  int si = segmentIndex(t);
  VectorX lu = u(t, derivative_order);
  for(int j = 0; j < spline_order_; j++) {
    ret[j] = lu.dot(basis_matrices_[si].col(j));
  }
}

VectorX BSpline::getLocalCumulativeBiVector(real_t t, int derivative_order) const
{
  VectorX bi = getLocalBiVector(t, derivative_order);
  int maxIndex = bi.rows() - 1;
  // tildeB(i) = np.sum(bi[i+1:]) :
  for(int i = 1; i <= maxIndex; i ++)
  {
    real_t sum = 0;
    for(int j = maxIndex; j > i; j--)
    {
      sum += bi[j];
    }
    bi[i] += sum;
  }
  if (derivative_order == 0)
  {
    bi[0] = 1; // the sum of k successive spline basis functions is always 1
  }
  else
  {
    bi[0] = 0;
  }
  return bi;
}

int BSpline::segmentIndex(real_t t) const
{
  std::pair<real_t,int> ui = computeTIndex(t);

  return basisMatrixIndexFromStartingKnotIndex(ui.second);
}

MatrixX BSpline::U(real_t t, int derivative_order) const
{
  VectorX uvec = u(t,derivative_order);
  int D = coefficients_.rows();
  MatrixX Umat = MatrixX::Zero(spline_order_ * D, D);

  for(int i = 0; i < D; i++)
  {
    Umat.block(i*spline_order_,i,spline_order_,1) = uvec;
  }

  return Umat;
}

VectorX BSpline::u(real_t t, int derivative_order) const
{
  std::pair<real_t,int> ui = computeUAndTIndex(t);

  return computeU(ui.first, ui.second, derivative_order);
}

MatrixX BSpline::Di(int segment_index) const
{
  int D = coefficients_.rows();
  MatrixX fullD = MatrixX::Zero(spline_order_*D, spline_order_*D);

  MatrixX subD = Dii(segment_index);

  for(int d = 0; d < D; d++)
  {
    fullD.block(d*spline_order_,d*spline_order_,spline_order_,spline_order_) = subD;
  }

  return fullD;
}

MatrixX BSpline::Dii(int segment_index) const
{
  CHECK_LE(segment_index, (int)basis_matrices_.size()) << "Out of range";
  CHECK_LT(0, (int)basis_matrices_.size()) << "Out of range";
  real_t t_0,t_1;
  std::tie(t_0,t_1) = timeInterval(segment_index);
  real_t dt = t_1 - t_0;

  real_t recip_dt = 0.0;
  if(dt > 0)
  {
    recip_dt = 1.0/dt;
  }
  MatrixX D = MatrixX::Zero(spline_order_,spline_order_);
  for(int i = 0; i < spline_order_ - 1; i++)
  {
    D(i,i+1) = (i+1.0) * recip_dt;
  }

  return D;
}

MatrixX BSpline::segmentQuadraticIntegral(const MatrixX& W,
                                          int segment_idx,
                                          int derivative_order) const
{
  int D = coefficients_.rows();
  //CHECK_GE(segmentIndex, (int)basisMatrices_.size()) << "Out of range";
  CHECK_LT(0, (int)basis_matrices_.size()) << "Out of range";
  CHECK_EQ(W.rows(), D)
      <<"W must be a square matrix the size of a single vector-valued coefficient";
  CHECK_EQ(W.cols(), D)
      << "W must be a square matrix the size of a single vector-valued coefficient";

  int N = D * spline_order_;
  MatrixX Q;// = MatrixX::Zero(N,N);
  MatrixX Dm = Dii(segment_idx);
  MatrixX V = Vi(segment_idx);
  MatrixX M = Mi(segment_idx);

  // Calculate the appropriate derivative version of V
  // using the matrix multiplication version of the derivative.
  for(int i = 0; i < derivative_order; i++)
  {
    V = (Dm.transpose() * V * Dm).eval();
  }

  MatrixX WV = MatrixX::Zero(N,N);

  for(int r = 0; r < D; r++)
  {
    for(int c = 0; c < D; c++)
    {
      CHECK_GE(1e-14, std::abs(W(r,c) - W(c,r))) << "W must be symmetric";
      //std::cout << "Size WV: " << WV.rows() << ", " << WV.cols() << std::endl;
      //std::cout << "Size V: " << V.rows() << ", " << V.cols() << std::endl;
      WV.block(spline_order_*r,
               spline_order_*c,
               spline_order_,
               spline_order_) = W(r,c) * V;
    }
  }

  Q = M.transpose() * WV * M;

  return Q;
}

MatrixX BSpline::segmentQuadraticIntegralDiag(const VectorX& Wdiag,
                                              int segment_idx,
                                              int derivative_order) const
{
  int D = coefficients_.rows();
  //CHECK_GE(segmentIndex, (int)basisMatrices_.size()) << "Out of range";
  CHECK_LT(0, (int)basis_matrices_.size()) << "Out of range";
  CHECK_EQ(Wdiag.size(), D) << "Wdiag must be the length of a single vector-valued coefficient";

  int N = D * spline_order_;
  MatrixX Q;// = MatrixX::Zero(N,N);
  MatrixX Dm = Dii(segment_idx);
  MatrixX V = Vi(segment_idx);
  MatrixX M = Mi(segment_idx);

  // Calculate the appropriate derivative version of V
  // using the matrix multiplication version of the derivative.
  for(int i = 0; i < derivative_order; i++)
  {
    V = (Dm.transpose() * V * Dm).eval();
  }

  MatrixX WV = MatrixX::Zero(N,N);

  for(int d = 0; d < D; d++)
  {
    //std::cout << "Size WV: " << WV.rows() << ", " << WV.cols() << std::endl;
    //std::cout << "Size V: " << V.rows() << ", " << V.cols() << std::endl;
    WV.block(spline_order_*d, spline_order_*d,spline_order_,spline_order_) = Wdiag(d) * V;
  }

  Q = M.transpose() * WV * M;

  return Q;
}

MatrixX BSpline::curveQuadraticIntegral(const MatrixX& W,
                                        int derivative_order) const
{
  int D = coefficients_.rows();
  CHECK_EQ(W.rows(), D)
      << "W must be a square matrix the size of a single vector-valued coefficient";
  CHECK_EQ(W.cols(), D)
      << "W must be a square matrix the size of a single vector-valued coefficient";
  int N = coefficients_.cols();

  MatrixX Q = MatrixX::Zero(D*N, D*N);

  int QiSize = spline_order_ * D;
  for(int s = 0; s < numValidTimeSegments(); s++)
  {
    Q.block(s*D,s*D,QiSize,QiSize) += segmentQuadraticIntegral(W, s, derivative_order);
  }

  return Q;
}

MatrixX BSpline::curveQuadraticIntegralDiag(const VectorX& Wdiag,
                                            int derivative_order) const
{
  int D = coefficients_.rows();
  CHECK_EQ(Wdiag.size(), D)
      << "Wdiag must be the length of a single vector-valued coefficient";
  int N = coefficients_.cols();

  MatrixX Q = MatrixX::Zero(D*N, D*N);

  int QiSize = spline_order_ * D;
  for(int s = 0; s < numValidTimeSegments(); s++)
  {
    Q.block(s*D,s*D,QiSize,QiSize) += segmentQuadraticIntegralDiag(Wdiag,
                                                                   s,
                                                                   derivative_order);
  }

  return Q;
}

int BSpline::coefficientVectorLength() const
{
  return coefficients_.rows() * coefficients_.cols();
}

void BSpline::initConstantSpline(real_t t_min,
                                 real_t t_max,
                                 int num_segments,
                                 const VectorX& constant)
{
  CHECK_GT(t_max, t_min) << "The max time is less than the min time";
  CHECK_GE(num_segments, 1) << "There must be at least one segment";
  CHECK_GE(constant.size(), 1)
      << "The constant vector must be of at least length 1";

  int K = numKnotsRequired(num_segments);
  int C = numCoefficientsRequired(num_segments);
  real_t dt = (t_max - t_min) / (real_t)num_segments;

  real_t minTime = t_min - (spline_order_ - 1)*dt;
  real_t maxTime = t_max + (spline_order_ - 1)*dt;
  VectorX knotVector = VectorX::LinSpaced(K,minTime,maxTime);
  // std::cout << "K: " << K << std::endl;
  // std::cout << "S: " << numSegments << std::endl;
  // std::cout << "segTime: " << t_min << ", " << t_max << std::endl;
  // std::cout << "dt: " << dt << std::endl;
  // std::cout << "time: " << minTime << ", " << maxTime << std::endl;
  // std::cout << "order: " << splineOrder_ << std::endl;
  // std::cout << knotVector.transpose() << std::endl;
  MatrixX coeff(constant.size(),C);
  for(int i = 0; i < C; i++)
  {
    coeff.col(i) = constant;
  }

  setKnotVectorAndCoefficients(knotVector,coeff);
}

int BSpline::numCoefficients() const
{
  return coefficients_.rows() * coefficients_.cols();
}

Eigen::Map<VectorX> BSpline::vvCoefficientVector(int i)
{
  CHECK_LE(i, coefficients_.cols()) << "Index out of range";
  CHECK_LT(0, coefficients_.cols()) << "Index out of range";
  return Eigen::Map<VectorX>(&coefficients_(0,i),coefficients_.rows());
}

Eigen::Map<const VectorX> BSpline::vvCoefficientVector(int i) const
{
  CHECK_LE(i, coefficients_.cols()) << "Index out of range";
  CHECK_LT(0, coefficients_.cols()) << "Index out of range";
  return Eigen::Map<const VectorX>(&coefficients_(0,i),coefficients_.rows());
}

int BSpline::numVvCoefficients() const
{
  return coefficients_.cols();
}

}  // namespace ze
