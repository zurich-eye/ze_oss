#include <ze/splines/bspline_pose_minimal.hpp>

// Bring in gtest
#include <ze/common/test_entrypoint.h>
#include <boost/tuple/tuple.hpp>
#include <ze/common/numerical_derivative.h>
#include <ze/common/transformation.h>
#include <ze/splines/operators.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

using namespace ze;

namespace ze {

// A wrapper around a bspline to numerically estimate Jacobians at a given
// instance of time and derivative order
template<class ROTATION>
class FixedTimeBSplinePoseMinimal
{
public:
  //! t: time of evaluation
  //! d: derivative order (0..splineOrder -1)
  //! v: homogeneous coordinates vector to transform (transformation Jacobians)
  FixedTimeBSplinePoseMinimal(
      BSplinePoseMinimal<ROTATION>* bs,
      FloatType t, int d, Vector4 v = Vector4::Zero())
    : bs_(bs)
    , t_(t)
    , d_(d)
    , v_(v)
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

  //! evaluate the transformation given a local coefficient vector
  //! strictly it is T * v that is evaluated and not T itself
  Vector4 transformation(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    Matrix4 value = bs_->transformation(t_);
    setCoefficientVector(old_c);

    return value * v_;
  }

  //! evaluate the inverse transformation given a local coefficient vector
  //! strictly it is T^1 * v that is evaluated and not T itself
  Vector4 inverseTransformation(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    Matrix4 value = bs_->inverseTransformation(t_);
    setCoefficientVector(old_c);

    return value * v_;
  }

  Vector3 linearAcceleration(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    Vector3 value = bs_->linearAccelerationAndJacobian(t_, NULL, NULL);
    setCoefficientVector(old_c);

    return value;
  }

  Vector3 angularVelocity(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    Vector3 value = bs_->angularVelocity(t_);
    setCoefficientVector(old_c);

    return value;
  }

  Vector3 angularVelocityBodyFrame(VectorX c)
  {
    VectorX old_c = coefficientVector();
    setCoefficientVector(c);
    Vector3 value = bs_->angularVelocityBodyFrame(t_);
    setCoefficientVector(old_c);

    return value;
  }

private:
  BSplinePoseMinimal<ROTATION>* bs_;
  FloatType t_;
  int d_;
  Vector4 v_;
};
} // namespace ze

TEST(BSplinePoseMinimalTestSuite, testCurveValueToTransformation)
{
  BSplinePoseMinimal<ze::kinematics::RotationVector> bs(3);

  Vector6 point = Vector6::Random();
  Matrix4 T = bs.curveValueToTransformation(point);

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(
                bs.transformationToCurveValue(T),
                point,
                1e-6));
}

// Check that the Jacobian calculation is correct.
TEST(BSplinePoseMinimalTestSuite, testBSplineTransformationJacobian)
{
  for(int order = 2; order < 10; order++)
  {
    // Create a two segment spline.
    BSplinePoseMinimal<ze::kinematics::RotationVector> bs(order);
    bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(VectorX::Random(6)),
                      bs.curveValueToTransformation(VectorX::Random(6)));
    bs.addPoseSegment(2.0,bs.curveValueToTransformation(VectorX::Random(6)));

    // Create a random homogeneous vector.
    Vector4 v = Vector4::Random() * 10.0;

    for(FloatType t = bs.t_min(); t <= bs.t_max(); t+= 0.413)
    {
      FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector> fixed_bs(
            &bs, t, 0, v);

      Eigen::Matrix<FloatType, Eigen::Dynamic, 1> point =
          fixed_bs.coefficientVector();
      MatrixX estJ =
          numericalDerivative<MatrixX, VectorX>(
            std::bind(
              &FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector>::transformation,
              &fixed_bs, std::placeholders::_1), point);

      MatrixX JT;
      Matrix4 T = bs.transformationAndJacobian(t, &JT);

      MatrixX J = ze::kinematics::boxMinus(T*v) * JT;

      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-6));

      // Try again with the lumped function.
      Vector4 v_n = bs.transformVectorAndJacobian(t, v, &J);
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(v_n, T*v, 1e-6));
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-6));

      return;
    }
  }
}

// Check that the Jacobian calculation is correct.
TEST(BSplinePoseMinimalTestSuite, testBSplineInverseTransformationJacobian)
{
  for(int order = 2; order < 10; order++) {
    // Create a two segment spline.
    BSplinePoseMinimal<ze::kinematics::RotationVector> bs(order);
    bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(VectorX::Random(6)),
                      bs.curveValueToTransformation(VectorX::Random(6)));
    bs.addPoseSegment(2.0,bs.curveValueToTransformation(VectorX::Random(6)));

    // Create a random homogeneous vector.
    Vector4 v = Vector4::Random() * 10.0;

    for(FloatType t = bs.t_min(); t <= bs.t_max(); t+= 0.413) {
      FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector> fixed_bs(
            &bs, t, 0, v);

      Eigen::Matrix<FloatType, Eigen::Dynamic, 1> point =
          fixed_bs.coefficientVector();
      MatrixX estJ =
          numericalDerivative<MatrixX, VectorX>(
            std::bind(
              &FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector>::inverseTransformation,
              &fixed_bs, std::placeholders::_1), point);

      MatrixX JT;
      MatrixX J;

      Matrix4 T = bs.inverseTransformationAndJacobian(t, &JT);

      J = ze::kinematics::boxMinus(T*v) * JT;
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-6));
    }
  }
}

TEST(BSplinePoseMinimalTestSuite, testBSplineAccelerationJacobian)
{
  for(int order = 2; order < 10; order++) {
    // Create a two segment spline.
    BSplinePoseMinimal<ze::kinematics::RotationVector> bs(order);
    bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(VectorX::Random(6)),
                      bs.curveValueToTransformation(VectorX::Random(6)));
    bs.addPoseSegment(2.0,bs.curveValueToTransformation(VectorX::Random(6)));

    for(FloatType t = bs.t_min(); t <= bs.t_max(); t+= 0.1) {
      MatrixX J;
      bs.linearAccelerationAndJacobian(t, &J, NULL);

      FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector> fixed_bs(
            &bs, t, 0);

      Eigen::Matrix<FloatType, Eigen::Dynamic, 1> point =
          fixed_bs.coefficientVector();
      MatrixX estJ =
          numericalDerivative<VectorX, VectorX>(
            std::bind(
              &FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector>::linearAcceleration,
              &fixed_bs, std::placeholders::_1), point);

      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-6));

    }
  }
}

TEST(BSplinePoseMinimalTestSuite, testBSplineAngularVelocityJacobian)
{
  for(int order = 2; order < 10; order++) {
    // Create a two segment spline.
    BSplinePoseMinimal<ze::kinematics::RotationVector> bs(order);
    bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(VectorX::Random(6)),
                      bs.curveValueToTransformation(VectorX::Random(6)));
    bs.addPoseSegment(2.0,bs.curveValueToTransformation(VectorX::Random(6)));

    for(FloatType t = bs.t_min(); t <= bs.t_max(); t+= 0.1) {
      MatrixX J;
      bs.angularVelocityAndJacobian(t, &J, NULL);

      FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector> fixed_bs(
            &bs, t, 0);

      Eigen::Matrix<FloatType, Eigen::Dynamic, 1> point =
          fixed_bs.coefficientVector();
      MatrixX estJ =
          numericalDerivative<VectorX, VectorX>(
            std::bind(
              &FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector>::angularVelocity,
              &fixed_bs, std::placeholders::_1), point);

      // opposite sign due to perturbation choice
      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, -estJ, 1e-6));

    }
  }
}

TEST(BSplinePoseMinimalTestSuite, testBSplineAngularVelocityBodyFrameJacobian)
{
  for(int order = 2; order < 10; order++) {
    // Create a two segment spline.
    BSplinePoseMinimal<ze::kinematics::RotationVector> bs(order);
    bs.initPoseSpline(0.0, 1.0, bs.curveValueToTransformation(VectorX::Random(6)),
                      bs.curveValueToTransformation(VectorX::Random(6)));
    bs.addPoseSegment(2.0,bs.curveValueToTransformation(VectorX::Random(6)));

    for(FloatType t = bs.t_min(); t <= bs.t_max(); t+= 0.1) {
      MatrixX J;
      bs.angularVelocityBodyFrameAndJacobian(t, &J, NULL);

      FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector> fixed_bs(
            &bs, t, 0);

      Eigen::Matrix<FloatType, Eigen::Dynamic, 1> point =
          fixed_bs.coefficientVector();
      MatrixX estJ =
          numericalDerivative<VectorX, VectorX>(
            std::bind(
              &FixedTimeBSplinePoseMinimal<ze::kinematics::RotationVector>::angularVelocityBodyFrame,
              &fixed_bs, std::placeholders::_1), point);

      EXPECT_TRUE(EIGEN_MATRIX_NEAR(J, estJ, 1e-6));

    }
  }
}

ZE_UNITTEST_ENTRYPOINT
