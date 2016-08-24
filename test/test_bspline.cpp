// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/splines/bspline.hpp>

// Bring in gtest
#include <ze/common/test_entrypoint.h>
#include <ze/common/numerical_derivative.h>
#include <ze/common/manifold.h>

namespace ze {

// A wrapper around a bspline to numerically estimate Jacobians at a given
// instance of time and derivative order
class FixedTimeBSpline
{
public:
  //! t: time of evaluation
  //! d: derivative order (0..splineOrder -1)
  FixedTimeBSpline(BSpline* bs, real_t t, int d)
    : bs_(bs), t_(t), d_(d)
  {
  }

  //! get the vector of active coefficients at the evaluation time
  VectorX coefficientVector()
  {
    return bs_->localCoefficientVector(t_);
  }

  //! set the coefficient vector at the evaluation time
  void setCoefficientVector(VectorX c)
  {
    return bs_->setLocalCoefficientVector(t_, c);
  }

  //! evaluate the splines given a local coefficient vector
  VectorX eval(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    VectorX value = bs_->evalD(t_, d_);
    setCoefficientVector(old_c);

    return value;
  }

private:
  BSpline* bs_;
  real_t t_;
  int d_;
};
} // namespace ze

// Check that the Jacobian calculation is correct.
TEST(SplineTestSuite, testBSplineJacobian)
{
  using namespace ze;

  const int segments = 2;
  for(int order = 2; order < 10; order++)
  {
    BSpline bs(order);
    int nk = bs.numKnotsRequired(segments);
    std::vector<real_t> knots;
    for (int i = 0; i < nk; ++i)
    {
      knots.push_back(i);
    }

    for (int dim = 1; dim < 4; ++dim)
    {
      int nc = bs.numCoefficientsRequired(segments);
      MatrixX C = MatrixX::Random(dim,nc);
      bs.setKnotsAndCoefficients(knots, C);
      for (int derivative = 0; derivative < order; ++derivative)
      {
        for (real_t t = bs.t_min(); t < bs.t_max(); t += 0.1)
        {

          FixedTimeBSpline fixed_bs(&bs, t, derivative);

          Eigen::Matrix<real_t, Eigen::Dynamic, 1> point =
              fixed_bs.coefficientVector();
          MatrixX estJ =
              numericalDerivative<MatrixX, VectorX>(
                std::bind(&FixedTimeBSpline::eval, &fixed_bs,
                          std::placeholders::_1), point);

          MatrixX J;
          VectorX v;
          std::tie(v,J) = bs.evalDAndJacobian(t, derivative);

          EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-5));
        }
      }
    }
  }
}

TEST(SplineTestSuite, testCoefficientMap)
{
  using namespace ze;

  const int order = 4;
  const int segments = 10;
  const int dim = 5;
  BSpline bs(order);
  int nk = bs.numKnotsRequired(segments);
  int nc = bs.numCoefficientsRequired(segments);

  std::vector<real_t> knots;
  for (int i = 0; i < nk; ++i)
  {
    knots.push_back(i);
  }

  MatrixX C = MatrixX::Random(dim,nc);
  bs.setKnotsAndCoefficients(knots, C);

  const MatrixX & CC = bs.coefficients();
  for (int i = 0; i < bs.numVvCoefficients(); ++i)
  {
    Eigen::Map<VectorX> m = bs.vvCoefficientVector(i);
    // Test pass by value...
    Eigen::Map<Eigen::Matrix<real_t, 5, 1> > m2=
        bs.fixedSizeVvCoefficientVector<dim>(i);
    for (int r = 0; r < m.size(); ++r)
    {
      ASSERT_TRUE( &m[r] == &CC(r,i) );
      ASSERT_TRUE( &m[r] == &m2[r] );
      m[r] = rand();
      ASSERT_EQ( m[r], CC(r,i) );
      ASSERT_EQ( m[r], m2[r] );
    }
  }
}

TEST(SplineTestSuite, testGetBi)
{
  using namespace ze;

  const int order = 4;
  const int segments = 10;
  const real_t startTime = 0;
  const real_t endTime = 5;
  const int numTimeSteps = 43;
  BSpline bs(order);
  bs.initConstantSpline(startTime, endTime, segments, VectorX::Zero(1));

  for (int i = 0; i <= numTimeSteps; ++i)
  {
    real_t t = startTime + (endTime - startTime) * ((real_t) i / numTimeSteps);
    VectorX localBiVector = bs.getLocalBiVector(t);

    EXPECT_NEAR(localBiVector.sum(), 1.0, 1e-10)
        << "the bis at a given time should always sum up to 1";

    VectorX biVector = bs.getBiVector(t);
    EXPECT_NEAR(localBiVector.sum(), 1.0, 1e-10)
        << "the bis at a given time should always sum up to 1";

    VectorX cumulativeBiVector = bs.getCumulativeBiVector(t);
    VectorX localCumulativeBiVector = bs.getLocalCumulativeBiVector(t);

    VectorXi localCoefficientVectorIndices =
        bs.localCoefficientVectorIndices(t);
    int firstIndex = localCoefficientVectorIndices[0];
    EXPECT_EQ(localCoefficientVectorIndices.size(), order)
        << "localCoefficientVectorIndices has to have exactly " << order << " entries";

    for (int j = 0; j < order; ++j)
    {
      EXPECT_EQ(localCoefficientVectorIndices[j], firstIndex + j)
          << "localCoefficientVectorIndices have to be successive";
    }

    for (int j = 0, n = biVector.size(); j < n; ++j)
    {
      if (j < firstIndex)
      {
        EXPECT_EQ(biVector[j], 0);
        EXPECT_EQ(cumulativeBiVector[j], 1);
      }
      else if (j < firstIndex + order)
      {
        EXPECT_EQ(biVector[j], localBiVector[j - firstIndex])
            << "localBiVector should be a slice of the biVector";
        EXPECT_EQ(cumulativeBiVector[j], localCumulativeBiVector[j - firstIndex])
            << "localCumulativeBiVector must be a slice of the cumulativeBiVector";
        EXPECT_NEAR(cumulativeBiVector[j],
                    localBiVector.segment(j - firstIndex, order - (j - firstIndex)).sum(),
                    1e-13)
            << "cumulativeBiVector must be the sum of the localBiVector"
            << "where it overlaps, but it is not at " << (j - firstIndex)
            << " (localBiVector=" << localBiVector << ")";
      }
      else
      {
        EXPECT_EQ(biVector[j], 0) << "at position " << j;
        EXPECT_EQ(cumulativeBiVector[j], 0) << "at position " << j;
      }
    }
  }
}

ZE_UNITTEST_ENTRYPOINT
