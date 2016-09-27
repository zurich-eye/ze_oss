// Copyright (c) 2014, Paul Furgale, Jérôme Maye and Jörn Rehder, Autonomous Systems Lab, ETH Zurich, Switzerland
// Copyright (c) 2014, Thomas Schneider, Skybotix AG, Switzerland
// Copyright (c) 2016, Luc Oth
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this list of
// conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this list of
// conditions and the following disclaimer in the documentation and/or other materials
// provided with the distribution.
//
// * All advertising materials mentioning features or use of this software must display the
// following acknowledgement: This product includes software developed by the
// Autonomous Systems Lab and Skybotix AG.
//
// * Neither the name of the Autonomous Systems Lab and Skybotix AG nor the names of its
// contributors may be used to endorse or promote products derived from this software
// without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTONOMOUS SYSTEMS LAB AND SKYBOTIX AG ''AS IS'' AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// the AUTONOMOUS SYSTEMS LAB OR SKYBOTIX AG BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
// OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

#include <ze/splines/bspline.hpp>
#include <ze/common/transformation.hpp>
#include <ze/splines/rotation_vector.hpp>
#include <ze/splines/operators.hpp>
#include <ze/common/macros.hpp>

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
    ZE_POINTER_TYPEDEFS(BSplinePoseMinimal);

    /**
     * Create a spline of the specified order. The resulting B-spline will
     * be a series of piecewise polynomials of degree splineOrder - 1.
     *
     * @param splineOrder The order of the spline.
     */
    BSplinePoseMinimal(int spline_order);

    ~BSplinePoseMinimal();

    Matrix4 transformation(real_t tk) const;
    Matrix4 transformationAndJacobian(
        real_t tk,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Matrix4 inverseTransformationAndJacobian(
        real_t tk,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Matrix4 inverseTransformation(real_t tk) const;


    Vector4 transformVectorAndJacobian(
        real_t tk,
        const Vector4& v,
        MatrixX* J = NULL,
        VectorXi* coefficient_indices = NULL) const;

    Vector3 position(real_t tk) const;

    Matrix3 orientation(real_t tk) const;
    Matrix3 orientationAndJacobian(
        real_t tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Matrix3 inverseOrientation(real_t tk) const;
    Matrix3 inverseOrientationAndJacobian(
        real_t tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 linearVelocity(real_t tk) const;
    Vector3 linearVelocityBodyFrame(real_t tk) const;

    Vector3 linearAcceleration(real_t tk) const;
    Vector3 linearAccelerationBodyFrame(real_t tk) const;
    Vector3 linearAccelerationAndJacobian(
        real_t tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 angularVelocity(real_t tk) const;
    Vector3 angularVelocityBodyFrame(real_t tk) const;
    Vector3 angularVelocityBodyFrameAndJacobian(
        real_t tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    Vector3 angularVelocityAndJacobian(
        real_t tk,
        MatrixX* J,
        VectorXi* coefficient_indices) const;

    //! takes the two transformation matrices at two points in time
    //! to construct a pose spline
    void initPoseSpline(
        real_t t0,
        real_t t1,
        const Matrix4& T_n_t0,
        const Matrix4& T_n_t);

    //! take the pose in the minimal parametrization to initializ the spline
    void initPoseSpline2(
        const VectorX& times,
        const Eigen::Matrix<real_t, 6, Eigen::Dynamic>& poses,
        int num_segments,
        real_t lambda);

    void initPoseSpline3(
        const VectorX& times,
        const Eigen::Matrix<real_t, 6, Eigen::Dynamic>& poses,
        int num_segments,
        real_t lambda);

    //! initialize a bspline given a vector of poses
    void initPoseSplinePoses(
        const VectorX& times,
        const std::vector<Matrix4>& poses,
        int num_segments,
        real_t lambda);

    void initPoseSplinePoses(
        const StampedTransformationVector& poses,
        int num_segments,
        real_t lambda);

    void addPoseSegment(
        real_t tk,
        const Matrix4& T_n_tk);

    void addPoseSegment2(
        real_t tk,
        const Matrix4& T_n_tk,
        real_t lambda);

    Matrix4 curveValueToTransformation(const VectorX& c) const;
    VectorX transformationToCurveValue(const Matrix4& T) const;

    Matrix4 curveValueToTransformationAndJacobian(
        const VectorX& c,
        MatrixX * J) const;
};

typedef BSplinePoseMinimal<ze::sm::RotationVector> BSplinePoseMinimalRotationVector;

}  // namespace ze
