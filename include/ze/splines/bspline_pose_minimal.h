// Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder,
// Autonomous Systems Lab, ETH Zurich, Switzerland
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

#pragma once

#include "bspline.h"
#include <ze/common/transformation.h>
#include <ze/splines/rotation_vector.h>
#include <ze/splines/operators.h>

namespace ze {

/**
 * @class BSpline
 *
 * A class to facilitate state estimation for vehicles in 3D space using B-Splines
 * The spline represents a pose with respect to some navigation frame
 * \f$ \mathbf F_n \f$.
 * The pose is represented as a 6d spline of a translational component and
 * a 3d minimal rotation parametrization.
 *
 */
template<class ROTATION>
class BSplinePoseMinimal : public BSpline
{
 public:

    /**
     * Create a spline of the specified order. The resulting B-spline will
     * be a series of piecewise polynomials of degree splineOrder - 1.
     *
     * @param splineOrder The order of the spline.
     */
    BSplinePoseMinimal(int spline_order);

    ~BSplinePoseMinimal();

    Matrix4 transformation(FloatType tk) const;
    Matrix4 transformationAndJacobian(
        FloatType tk,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Matrix4 inverseTransformationAndJacobian(
        FloatType tk,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Matrix4 inverseTransformation(FloatType tk) const;


    Vector4 transformVectorAndJacobian(
        FloatType tk,
        const Vector4& v,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Vector3 position(FloatType tk) const;

    Matrix3 orientation(FloatType tk) const;
    Matrix3 orientationAndJacobian(
        FloatType tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Matrix3 inverseOrientation(FloatType tk) const;
    Matrix3 inverseOrientationAndJacobian(
        FloatType tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 linearVelocity(FloatType tk) const;
    Vector3 linearVelocityBodyFrame(FloatType tk) const;

    Vector3 linearAcceleration(FloatType tk) const;
    Vector3 linearAccelerationBodyFrame(FloatType tk) const;
    Vector3 linearAccelerationAndJacobian(
        FloatType tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 angularVelocity(FloatType tk) const;
    Vector3 angularVelocityBodyFrame(FloatType tk) const;
    Vector3 angularVelocityBodyFrameAndJacobian(
        FloatType tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 angularVelocityAndJacobian(
        FloatType tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    //! takes the two transformation matrices at two points in time
    //! to construct a pose spline
    void initPoseSpline(FloatType t0,
                        FloatType t1,
                        const Matrix4& T_n_t0,
                        const Matrix4& T_n_t);

    //! take the pose in the minimal parametrization to initializ the spline
    void initPoseSpline2(const VectorX& times,
                         const Eigen::Matrix<FloatType, 6, Eigen::Dynamic>& poses,
                         int num_segments,
                         FloatType lambda);
    void initPoseSpline3(const VectorX& times,
                         const Eigen::Matrix<FloatType, 6, Eigen::Dynamic>& poses,
                         int num_segments,
                         FloatType lambda);

    //! initialize a bspline given a vector of poses
    void initPoseSplinePoses(const VectorX& times,
                            const std::vector<Matrix4>& poses,
                            int num_segments,
                            FloatType lambda);

    void addPoseSegment(FloatType tk, const Matrix4& T_n_tk);
    void addPoseSegment2(FloatType tk,
                         const Matrix4& T_n_tk,
                         FloatType lambda);

    Matrix4 curveValueToTransformation(const VectorX& c) const;
    VectorX transformationToCurveValue(const Matrix4& T) const;

    Matrix4 curveValueToTransformationAndJacobian(
        const VectorX& c,
        MatrixX * J) const;
};

typedef BSplinePoseMinimal<ze::sm::RotationVector> BSplinePoseMinimalRotationVector;

}  // namespace ze
