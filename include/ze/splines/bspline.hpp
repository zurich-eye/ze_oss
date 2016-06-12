// Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland
// Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland
// Copyright (c) 2016, Luc Oth
// All rights reserved.
//
// Adopted from https://github.com/ethz-asl/kalibr/ (2016)
// BSD Licensed
//
// Changes wrt. original:
//  _ remove dependency on Schweizer-Messer toolbox
//  _ remove dependency on sparse_block_matrix (drop support for sparse
//    initialization)
//
// WARNING: Do NOT use as-is in production code.
// @todo The code itself was initially designed purely for research and before
//       actually using it in production code, we should properly verify its
//       capabilities. One major part is the efficiency which isn't perfect at
//       all (pure Eigen::Dynamic implementations, a bunch of duplicate
//       evaluations for readability and some debuggng stuff)

#pragma once

#include <vector>
#include <Eigen/Core>
#include <ze/common/logging.hpp>
#include <ze/common/types.h>

namespace ze_splines {

// Define a generic expression of a matrix where all coefficients
// are defined by a functor.
// A BiVector is a vector that holds the values of cumulative basis functions
// and extends the definition to the outside of its definition:
// if before the segment where it is defined: 0
// after the segment: fixed to end_value
class BiVector;
}  // namespace ze_splines

namespace Eigen {
namespace internal {
template<>

struct functor_traits<ze_splines::BiVector>
{
  enum { Cost = 1, PacketAccess = false, IsRepeatable = true };
};
}  // namespace internal
}  // namespace Eigen

namespace ze_splines {

class BiVector {
  enum { Cost = 1, PacketAccess = false, IsRepeatable = true };
private:
  const int startIndex_;
  const ze::FloatType endValue_;
  const ze::VectorX localBi_;

public:
  BiVector(int startIndex, const ze::VectorX& localBi, ze::FloatType endValue)
    : startIndex_(startIndex)
    , endValue_(endValue)
    , localBi_(localBi)
  {};

  ze::FloatType operator() (int i, int j = 0) const {
    i -= startIndex_;
    if(i < 0)
    {
      return endValue_;
    }
    if(i >= (localBi_.rows()))
    {
      return 0;
    }
    return  i >= 0 ? localBi_(i): 0;
  }
};

} // namespace ze_splines

namespace ze {

/**
 * @class BSpline
 *
 * A class to facilitate state estimation for vechicles in 3D space using B-Splines
 *
 */
class BSpline
{
public:

  /**
   * Create a spline of the specified order. The resulting B-spline will
   * be a series of piecewise polynomials of degree splineOrder - 1.
   *
   * @param splineOrder The order of the spline.
   */
  BSpline(int spline_order);

  /**
   * A destructor.
   *
   */
  ~BSpline();

  /**
   *
   * @return The order of the spline
   */
  int spline_order() const;

  /**
   *
   * @return The degree of polynomial used by the spline.
   */
  int polynomialDegree() const;

  /**
   *
   * @return the minimum number of knots required to have at least one valid
   * time segment.
   */
  int minimumKnotsRequired() const;

  /**
   *
   * @return the dimension of the bspline (only valid after initialization)
   */
  int dimension() const;

  /**
   * @param numTimeSegments the number of time segments required
   * @return the number of coefficients required for a specified number of
   * valid time segments.
   */
  int numCoefficientsRequired(int num_time_segments) const;

  /**
   * @param numTimeSegments the number of time segments required
   * @return the number of knots required for a specified number of valid
   * time segments.
   */
  int numKnotsRequired(int num_time_segments) const;

  /**
   *
   * @param numKnots the number of knots.
   * @return the number of valid time segments for a given number of knots.
   */
  int numValidTimeSegments(int num_knots) const;

  /**
   *
   * @return the number of valid time segments for a given for the current
   * knot sequence.
   */
  int numValidTimeSegments() const;

  /**
   * Return the basis matrix active on the \f$i^{\textup{th}}\f$ time segment.
   *
   * @param i The index of the time segment.
   *
   * @return The basis matrix active on the time segment.
   */
  const MatrixX& basisMatrix(int i) const;

  /**
   *
   * @return the time interval that the spline is well-defined on
   * [t_min(), t_max()]
   */
  std::pair<FloatType, FloatType> timeInterval() const;

  /**
   * Return the time interval of a single spline segment.
   *
   * @param i The index of the time segment
   *
   * @return the time interval of the ith spline segment.
   */
  std::pair<FloatType, FloatType> timeInterval(int i) const;

  /**
   * Set the knots and coefficients of the spline. Each column of the
   * coefficient matrix
   * is interpreted as a single, vector-valued spline coefficient.
   *
   * @param knots        A non-decreasing knot sequence.
   * @param coefficients A set of spline coefficients.
   */
  void setKnotsAndCoefficients(const std::vector<FloatType>& knots,
                               const MatrixX& coefficients);

  /**
   * Set the knots and coefficients of the spline. Each column of the
   * coefficient matrix is interpreted as a single, vector-valued spline
   * coefficient.
   *
   * @param knots        A non-decreasing knot sequence.
   * @param coefficients A set of spline coefficients.
   */
  void setKnotVectorAndCoefficients(const VectorX& knots,
                                    const MatrixX& coefficients);

  /**
   * Sets the coefficient matrix from the stacked vector of coefficients.
   *
   * @param coefficients The stacked vector of coefficients.
   */
  void setCoefficientVector(const VectorX& coefficients);

  /**
   *
   * @return The stacked vector of coefficients.
   */
  VectorX coefficientVector();

  /**
   * Sets the matrix of coefficients.
   *
   * @param coefficients
   */
  void setCoefficientMatrix(const MatrixX& coefficients);

  /**
   * @return the current knot vector.
   */
  const std::vector<FloatType> knots() const;

  /**
   * @return the current knot vector.
   */
  VectorX knotVector() const;

  /**
   * @return the current spline coefficient matrix. Each column of
   * the coefficient matrix is interpreted as a single, vector-valued
   * spline coefficient.
   */
  const MatrixX& coefficients() const;

  /**
   *
   * @return The number of total coefficients the spline currently uses
   */
  int numCoefficients() const;

  /**
   * This is equivalent to spline.coefficients().cols()
   *
   * @return The number of vector-valued coefficient columns the spline
   * currently uses
   */
  int numVvCoefficients() const;

  /**
   *
   * @return The minimum time that the spline is well-defined on.
   */
  FloatType t_min() const;

  /**
   *
   * @return The maximum time that the spline is well-defined on. Because B-splines
   *         are defined on half-open intervals, the spline curve is well defined up
   *         to but not including this time.
   */
  FloatType t_max() const;

  /**
   * Evaluate the spline curve at the time t.
   *
   * @param t The time to evaluate the spline curve
   *
   * @return The value of the spline curve at the time t.
   */
  VectorX eval(FloatType t) const;

  /**
   * Evaluate the derivative of the spline curve at time t.
   *
   * @param t The time to evaluate the spline derivative.
   * @param derivativeOrder The order of the derivative. This must be >= 0
   *
   * @return The value of the derivative of the spline curve evaluated at t.
   */
  VectorX evalD(FloatType t, int derivative_order) const;

  /**
   * Evaluate the derivative of the spline curve at time t and retrieve the Jacobian
   * of the value with respect to small changes in the paramter vector. The Jacobian
   * only refers to the local parameter vector. The indices of the local parameters with
   * respect to the full paramter vector can be retrieved using localCoefficientVectorIndices().
   *
   * @param t The time to evaluate the spline derivative.
   * @param derivativeOrder The order of the derivative. This must be >= 0
   *
   * @return The value of the derivative of the spline curve evaluated at t and the Jacobian.
   */
  std::pair<VectorX, MatrixX> evalDAndJacobian(FloatType t,
                                               int derivative_order) const;

  /**
   * Evaluate the derivative of the spline curve at time t and retrieve the Jacobian
   * of the value with respect to small changes in the paramter vector. The Jacobian
   * only refers to the local parameter vector.
   *
   * @param t The time to evaluate the spline derivative.
   * @param derivativeOrder The order of the derivative. This must be >= 0
   * @param a pointer to the Jacobian matrix to fill in
   * @param a pointer to an int vector that will be filled with the local coefficient indices
   *
   * @return The value of the derivative of the spline curve evaluated at t.
   */
  VectorX evalDAndJacobian(FloatType t,
                           int derivative_order,
                           MatrixX * Jacobian,
                           VectorXi * coefficient_indices) const;

   /**
   * Get the local basis matrix evaluated at the time \f$ t \f$.
   * For vector-valued spline coefficients of dimension \f$ D \f$
   * and a B-spline of order $S$, this matrix will be \f$ D \times SD \f$
   *
   * @param t The time to evaluate the local basis matrix.
   * @param derivativeOrder The derivative order to return (0 is no derivative)
   *
   * @return The local basis matrix evaluated at time \f$ t \f$
   */
  MatrixX Phi(FloatType t, int derivative_order = 0) const;

  /**
   * Get the local basis matrix evaluated at the time \f$ t \f$.
   * For vector-valued spline coefficients of dimension \f$ D \f$
   * and a B-spline of order $S$, this matrix will be \f$ D \times SD \f$
   *
   * @param t The time to evaluate the local basis matrix.
   * @param derivativeOrder The derivative order to return (0 is no derivative)
   *
   * @return The local basis matrix evaluated at time \f$ t \f$
   */
  MatrixX localBasisMatrix(FloatType t, int derivative_order = 0) const;

  /**
   * Get the local coefficient matrix evaluated at the time \f$ t \f$.
   * For vector-valued spline coefficients of dimension \f$ D \f$
   * and a B-spline of order $S$, this matrix will be \f$ D \times S \f$.
   * Each column of the resulting matrix corresponds to a single vector-valued
   * coefficient.
   *
   * @param t The time being queried
   *
   * @return The local coefficient matrix active at time \f$ t \f$
   */
  MatrixX localCoefficientMatrix(FloatType t) const;

  /**
   * Return a map to a single coefficient column.
   * This allows the user to pass around what is essentially a pointer
   * to a single column in the coefficient matrix.
   *
   * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
   *
   * @return A map to column i of the coefficient matrix.
   */
  Eigen::Map<VectorX> vvCoefficientVector(int i);

  /**
   * Return a map to a single coefficient column.
   * This allows the user to pass around what is essentially a pointer
   * to a single column in the coefficient matrix.
   *
   * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
   *
   * @return A map to column i of the coefficient matrix.
   */
  Eigen::Map<const VectorX> vvCoefficientVector(int i) const;

  /**
   * Return a map to a single coefficient column.
   * This allows the user to pass around what is essentially a pointer
   * to a single column in the coefficient matrix.
   *
   * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
   *
   * @return A map to column i of the coefficient matrix.
   */
  template<int D>
  Eigen::Map<Eigen::Matrix<FloatType, D, 1> > fixedSizeVvCoefficientVector(int i)
  {
    CHECK_EQ(D,coefficients_.rows())
        << "Size mismatch between requested vector size and actual vector size";
    CHECK_LE(0, coefficients_.cols()) << "Index out of range";
    CHECK_LE(i, coefficients_.cols()) << "Index out of range";

    return Eigen::Map<Eigen::Matrix<FloatType, D, 1> >(&coefficients_(0,i),coefficients_.rows());
  }

  /**
   * Return a map to a single coefficient column.
   * This allows the user to pass around what is essentially a pointer
   * to a single column in the coefficient matrix.
   *
   * @param i The column of the coefficient matrix to return. \f$ 0 \le i < \f$ coefficients().cols()
   *
   * @return A map to column i of the coefficient matrix.
   */
  template<int D>
  Eigen::Map<const Eigen::Matrix<FloatType, D, 1> > fixedSizeVvCoefficientVector(int i) const
  {
    CHECK_EQ(D,coefficients_.rows())
        << "Size mismatch between requested vector size and actual vector size";
    CHECK_LE(0, coefficients_.cols()) << "Index out of range";
    CHECK_LE(i, coefficients_.cols()) << "Index out of range";

    return Eigen::Map< const Eigen::Matrix<FloatType, D, 1> >(
          &coefficients_(0,i), coefficients_.rows());
  }

  /**
   * Get the local coefficient vector evaluated at the time \f$ t \f$.
   * For vector-valued spline coefficients of dimension \f$ D \f$
   * and a B-spline of order $S$, this vector will be \f$ SD \times 1 \f$
   * Evaluating the B-spline at time t, eval(t,O) is equivalent to evaluating
   * Phi(t,O) * localCoefficientVector(t)
   *
   * @param t The time being queried
   *
   * @return The local coefficient vector active at time \f$ t \f$
   */
  VectorX localCoefficientVector(FloatType t) const;

  /**
   * Update the local coefficient vector
   *
   * @param t The time used to select the local coefficients.
   * @param c The local coefficient vector.
   */
  void setLocalCoefficientVector(FloatType t, const VectorX& c);

  /**
   * Get the indices of the local coefficients active at time t.
   *
   * @param t The time being queried.
   *
   * @return The indices of the local coefficients active at time t.
   */
  VectorXi localCoefficientVectorIndices(FloatType t) const;

  /**
   * Get the indices of the local vector-valued coefficients active at time t.
   *
   * @param t The time being queried.
   *
   * @return The indices of the local vector-valued coefficients active at time t.
   */
  VectorXi localVvCoefficientVectorIndices(FloatType t) const;

  int coefficientVectorLength() const;

  /**
   * Initialize a spline from two times and two positions. The spline will be initialized to
   * have one valid time segment \f$[t_0, t_1)\f$ such that \f$\mathbf b(t_0) = \mathbf p_0\f$,
   * \f$\mathbf b(t_1) = \mathbf p_1\f$,
   * \f$\dot{\mathbf b}(t_0) = \frac{\mathbf{p_1} - \mathbf p_0}{t_1 - t_0}\f$, and
   * \f$\dot{\mathbf b}(t_1) = \frac{\mathbf{p_1} - \mathbf p_0}{t_1 - t_0}\f$.
   *
   * @param t_0 The start of the time interval.
   * @param t_1 The end of the time interval
   * @param p_0 The position at the start of the time interval.
   * @param p_1 The position at the end of the time interval.
   */
  void initSpline(FloatType t_0,
                  FloatType t_1,
                  const VectorX& p_0,
                  const VectorX& p_1);

  //! Spline initialization version 2.
  void initSpline2(const VectorX& times,
                   const MatrixX& interpolation_points,
                   int num_segments,
                   FloatType lambda);

  //! Spline initialization version 3.
  void initSpline3(const VectorX& times,
                   const MatrixX& interpolation_points,
                   int num_segments,
                   FloatType lambda);

  /**
   * Add a curve segment that interpolates the point p, ending at time t.
   *
   * If the new time corresponds with the first knot past the end of the curve,
   * the existing curve is perfectly preserved. Otherwise, the existing curve
   * will interpolate its current position at the current endpoint and the new
   * position at the new endpoint but will not necessarily match the last segment
   * exactly.
   *
   * @param t The time of the point to interpolate. This must be greater than t_max()
   * @param p The point to interpolate at time t.
   */
  void addCurveSegment(FloatType t, const VectorX& p);

  /**
   * Add a curve segment that interpolates the point p, ending at time t.
   *
   * If the new time corresponds with the first knot past the end of the curve,
   * the existing curve is perfectly preserved. Otherwise, the existing curve
   * will interpolate its current position at the current endpoint and the new
   * position at the new endpoint but will not necessarily match the last segment
   * exactly.
   *
   * @param t The time of the point to interpolate. This must be greater than t_max()
   * @param p The point to interpolate at time t.
   * @param lambda a smoothness parameter. Higher for more smooth.
   */
  void addCurveSegment2(FloatType t, const VectorX& p, FloatType lambda);

  /**
   * Removes a curve segment from the left by removing one knot and one coefficient vector.
   * After calling this function, the curve will have one fewer segment. The new minimum
   * time will be timeInterval(0).first
   *
   */
  void removeCurveSegment();

  /**
   * Get the \f$ \mathbf V_i \f$ matrix associated with the integral over the segment.
   *
   * @param segmentIndex
   *
   * @return the \f$ \mathbf V_i \f$ matrix
   */
  MatrixX Vi(int segmentIndex) const;

  VectorX evalIntegral(FloatType t1, FloatType t2) const;
  inline VectorX evalI(FloatType t1, FloatType t2) const
  {
    return evalIntegral(t1, t2);
  }

  MatrixX Mi(int segment_index) const;
  MatrixX Bij(int segment_index, int column_index) const;
  MatrixX U(FloatType t, int derivative_order) const;
  VectorX u(FloatType t, int derivative_order) const;
  int segmentIndex(FloatType t) const;
  MatrixX Dii(int segment_index) const;
  MatrixX Di(int segment_index) const;

  /**
   * Get the b_i(t) for i in localVvCoefficientVectorIndices
   * (@see #localVvCoefficientVectorIndices).
   *
   * @param t The time being queried.
   *
   * @return [b_i(t) for i in localVvCoefficientVectorIndices].
   *
   */
  VectorX getLocalBiVector(FloatType t, int derivative_order = 0) const;
  void getLocalBiInto(FloatType t, VectorX& ret, int derivative_order = 0) const;

  /**
   * Get the cumulative (tilde) b_i(t) for i in localVvCoefficientVectorIndices
   * (@see #localVvCoefficientVectorIndices).
   *
   * @param t The time being queried.
   *
   * @return [tilde b_i(t) for i in localVvCoefficientVectorIndices].
   *
   */
  VectorX getLocalCumulativeBiVector(FloatType t, int derivative_order = 0) const;

  Eigen::CwiseNullaryOp<ze_splines::BiVector, VectorX>
  getBiVector(FloatType t) const
  {
    return Eigen::CwiseNullaryOp<ze_splines::BiVector, VectorX>(
          numValidTimeSegments(), 1,
          ze_splines::BiVector(segmentIndex(t), getLocalBiVector(t), 0));
  }
  Eigen::CwiseNullaryOp<ze_splines::BiVector, VectorX>
  getCumulativeBiVector(FloatType t) const
  {
    return Eigen::CwiseNullaryOp<ze_splines::BiVector, VectorX>(
          numValidTimeSegments(), 1,
          ze_splines::BiVector(segmentIndex(t),getLocalCumulativeBiVector(t), 1));
  }

  MatrixX segmentQuadraticIntegral(const MatrixX& W,
                                   int segment_index,
                                   int derivative_order) const;
  MatrixX segmentQuadraticIntegralDiag(const VectorX& Wdiag,
                                       int segment_index,
                                       int derivative_order) const;
  MatrixX curveQuadraticIntegral(const MatrixX& W,int derivative_order) const;
  MatrixX curveQuadraticIntegralDiag(const VectorX& Wdiag, int derivative_order) const;

  void initConstantSpline(FloatType t_min, FloatType t_max,
                          int num_segments, const VectorX& constant);

protected:
  /**
   * An internal function to find the segment of the knot sequence
   * that the time t falls in. The function returns the
   * value \f$ u = \frac{t - t_i}{t_{i+1} - t_i} \f$ and the index \f$i\f$
   *
   * @param t The time being queried.
   *
   * @return A pair with the first value \f$ u = \frac{t - t_i}{t_{i+1} - t_i} \f$
   * and the second value the index \f$i\f$
   */
  std::pair<FloatType,int> computeUAndTIndex(FloatType t) const;

  /**
   * An internal function to find the segment of the knot sequence
   * that the time t falls in. The function returns the width of the
   * knot segment \f$ \Delta t_i = t_{i+1} - t_i \f$ and the index \f$i\f$
   *
   * @param t The time being queried.
   *
   * @return A pair with the first value \f$ \Delta t_i = t_{i+1} - t_i \f$
   * and the second value the index \f$i\f$
   */
  std::pair<FloatType,int> computeTIndex(FloatType t) const;

  /**
   * Compute the vector \f$ \mathbf u(t) \f$ for a spline of
   * order \f$ S \f$, this is an \f$ S \times 1 \f$ vector.
   *
   * At derivative order 0 (no derivative), this vector is
   * \f$ \mathbf u(t) = \left [ 1 \; u(t) \; u(t)^2 \; \dots \; u(t)^{S-1} \right ]^T \f$
   *
   * For higher derivative order \f$ M \f$, the vector returned is
   * \f$ \mathbf u^{(M)}(t) = \frac{\partial^{(M)}\mathbf u(t)}{\partial t^{(M)}}
   *
   * @param uval the value \f$ u(t) \f$
   * @param segmentIndex
   * @param derivativeOrder
   *
   * @return
   */
  VectorX computeU(FloatType uval, int segment_index, int derivative_order) const;

  int basisMatrixIndexFromStartingKnotIndex(int starting_knot_index) const;
  int startingKnotIndexFromBasisMatrixIndex(int basis_matrix_index) const;
  const MatrixX& basisMatrixFromKnotIndex(int knot_index) const;

  /**
   * Throws an exception if the knot sequence is not non-decreasing.
   *
   * @param knots The knot sequence to verify.
   */
  void verifyKnotSequence(const std::vector<FloatType>& knots);

  /**
   * Initialize the basis matrices based on the current knot sequence.
   * There is one basis matrix for each valid time segment defined by the spline.
   *
   * Implemented using the recursive basis matrix algorithm from
   * Qin, Kaihuai, General matrix representations for B-splines,
   * The Visual Computer (2000) 16:177–186
   *
   */
  void initializeBasisMatrices();

  /**
   * The recursive function used to implement the recursive basis matrix algorithm from
   * Qin, Kaihuai, General matrix representations for B-splines,
   * The Visual Computer (2000) 16:177–186
   *
   * @param k The order of the matrix requested.
   * @param i The time segment of the basis matrix
   *
   * @return
   */
  MatrixX M(int k, int i);

  /**
   * A helper function for producing the M matrices. Defined in
   * Qin, Kaihuai, General matrix representations for B-splines,
   * The Visual Computer (2000) 16:177–186
   *
   */
  FloatType d_0(int k, int i, int j);

  /**
   * A helper function for producing the M matrices. Defined in
   * Qin, Kaihuai, General matrix representations for B-splines,
   * The Visual Computer (2000) 16:177–186
   *
   */
  FloatType d_1(int k, int i, int j);

  /// The order of the spline.
  int spline_order_;

  /// The knot sequence used by the B-spline.
  std::vector<FloatType> knots_;

  /// The coefficient matrix used by the B-Spline. Each column can be seen as a
  /// single vector-valued spline coefficient.
  /// This is stored explicityl in column major order to ensure that each column (i.e.
  /// a single vector-valued spline coefficient) is stored in contiguous memory. This
  /// allows one to, for example, map a single spline coefficient using the Eigen::Map type.
  Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor> coefficients_;

  /// The basis matrices for each time segment the B-spline is defined over.
  std::vector<MatrixX> basis_matrices_;
};
} // namespace ze
