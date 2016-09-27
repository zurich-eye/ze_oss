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

#include <ze/splines/bspline_pose_minimal.hpp>

#include <ze/common/time_conversions.hpp>

namespace ze {

template<class RP>
BSplinePoseMinimal<RP>::BSplinePoseMinimal(int spline_order)
  : BSpline(spline_order)
{
}

template<class RP>
BSplinePoseMinimal<RP>::~BSplinePoseMinimal()
{
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::transformation(real_t tk) const
{
  return curveValueToTransformation(eval(tk));
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::transformationAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  MatrixX JS;
  VectorX p;
  p = evalDAndJacobian(tk, 0, &JS, coefficient_indices);

  MatrixX JT;
  Matrix4 T = curveValueToTransformationAndJacobian(p, &JT);

  if(J)
  {
    *J = JT * JS;
  }

  return T;
}

template<class RP>
Matrix3 BSplinePoseMinimal<RP>::orientationAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  Matrix3 C;
  MatrixX JS;
  VectorX p;
  p = evalDAndJacobian(tk,0,&JS, coefficient_indices);

  Matrix3 S;

  RP rp(Vector3(p.tail<3>()));
  C = rp.getRotationMatrix();
  S = rp.toSMatrix();

  MatrixX JO = MatrixX::Zero(3,6);
  JO.block(0,3,3,3) = S;
  if(J)
  {
    *J = JO * JS;
  }

  return C;
}

template<class RP>
Matrix3 BSplinePoseMinimal<RP>::inverseOrientationAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  Matrix3 C;
  MatrixX JS;
  VectorX p;
  p = evalDAndJacobian(tk, 0, &JS, coefficient_indices);

  Matrix3 S;
  RP rp(Vector3(p.tail<3>()));
  C = rp.getRotationMatrix().transpose();
  S = rp.toSMatrix();

  MatrixX JO = MatrixX::Zero(3,6);
  JO.block(0,3,3,3) = S;
  if(J)
  {
    *J = -C * JO * JS;
  }

  return C;
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::inverseTransformationAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  MatrixX JS;
  VectorX p;
  p = evalDAndJacobian(tk,0,&JS, coefficient_indices);

  MatrixX JT;
  Matrix4 T = curveValueToTransformationAndJacobian( p, &JT );
  // Invert the transformation.
  T.topLeftCorner<3,3>().transposeInPlace();
  T.topRightCorner<3,1>() = (-T.topLeftCorner<3,3>()
                             * T.topRightCorner<3,1>()).eval();

  if(J)
  {
    // The "box times" is the linearized transformation way of
    // inverting the jacobian.
     *J = -ze::sm::boxTimes(T) * JT * JS;
  }

  if(coefficient_indices)
  {
    *coefficient_indices = localCoefficientVectorIndices(tk);
  }

  return T;
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::inverseTransformation(real_t tk) const
{
  Matrix4 T = curveValueToTransformation(eval(tk));
  T.topLeftCorner<3,3>().transposeInPlace();
  T.topRightCorner<3,1>() = (-T.topLeftCorner<3,3>() * T.topRightCorner<3,1>()).eval();
  return T;
}

template<class RP>
Vector4 BSplinePoseMinimal<RP>::transformVectorAndJacobian(
    real_t tk,
    const Vector4& v_tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  MatrixX JT;
  Matrix4 T_n_vk = transformationAndJacobian(tk, &JT, coefficient_indices);
  Vector4 v_n = T_n_vk * v_tk;

  if(J)
  {
     *J = ze::sm::boxMinus(v_n) * JT;
  }

  return v_n;
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::position(real_t tk) const
{
  Matrix61 v = eval(tk);
  return v.head<3>();
}

template<class RP>
Matrix3 BSplinePoseMinimal<RP>::orientation(real_t tk) const
{
  Matrix61 v = eval(tk);
  return RP(Vector3(v.tail<3>())).getRotationMatrix();
}

template<class RP>
Matrix3 BSplinePoseMinimal<RP>::inverseOrientation(real_t tk) const
{
  Matrix61 v = eval(tk);
  return RP(Vector3(v.tail<3>())).getRotationMatrix().transpose();
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::linearVelocity(real_t tk) const
{
  Matrix61 v = evalD(tk, 1);
  return v.head<3>();
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::linearVelocityBodyFrame(real_t tk) const
{
  Matrix61 r = evalD(tk, 0);
  Matrix61 v = evalD(tk, 1);
  Matrix3 C_wb = RP(Vector3(r.tail<3>())).getRotationMatrix();
  return C_wb.transpose() * v.head<3>();
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::linearAcceleration(real_t tk) const
{
  Matrix61 v = evalD(tk, 2);
  return v.head<3>();
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::linearAccelerationBodyFrame(real_t tk) const
{
  VectorX r = evalD(tk, 0);
  Matrix61 v = evalD(tk, 2);
  Matrix3 C_wb = RP(Vector3(r.tail<3>())).getRotationMatrix();
  return C_wb.transpose() * v.head<3>();
}

template<class RP>
Vector3 BSplinePoseMinimal<RP>::linearAccelerationAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{

  Matrix61 v = evalDAndJacobian(tk, 2, J, coefficient_indices);
  Vector3 a = v.head<3>();
  if(J)
  {
    J->conservativeResize(3,J->cols());
  }
  return a;
}

// \omega_w_{b,w} (angular velocity of the body frame as seen from the world
// frame, expressed in the world frame)
template<class RP>
Vector3 BSplinePoseMinimal<RP>::angularVelocity(real_t tk) const
{
  Vector3 omega;
  VectorX r = evalD(tk,0);
  VectorX v = evalD(tk,1);

  // \omega = S(\bar \theta) \dot \theta
  RP rp(Vector3(r.tail<3>()));
  Matrix3 S = rp.toSMatrix();

  omega = -S * v.tail<3>();
  return omega;
}

// \omega_b_{w,b} (angular velocity of the world frame as seen from the body
// frame, expressed in the body frame)
template<class RP>
Vector3 BSplinePoseMinimal<RP>::angularVelocityBodyFrame(real_t tk) const
{
  Vector3 omega;
  VectorX r = evalD(tk,0);
  VectorX v = evalD(tk,1);
  Matrix3 S;
  RP rp(Vector3(r.tail<3>()));
  Matrix3 C_w_b = rp.getRotationMatrix();

  // \omega = S(\bar \theta) \dot \theta
  S = rp.toSMatrix();
  omega = -C_w_b.transpose() * S * v.tail<3>();

  return omega;
}

// \omega_b_{w,b} (angular velocity of the world frame as seen from the body
// frame, expressed in the body frame)
template<class RP>
Vector3 BSplinePoseMinimal<RP>::angularVelocityBodyFrameAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{
  Vector3 omega;
  Vector3 p;
  Vector3 pdot;
  MatrixX Jp;
  MatrixX Jpdot;
  Matrix61 v = evalDAndJacobian(tk, 0, &Jp, NULL);
  Matrix61 vdot = evalDAndJacobian(tk, 1, &Jpdot, coefficient_indices);
  p = v.tail<3>();
  pdot = vdot.tail<3>();

  MatrixX Jr;
  Matrix3 C_w_b = inverseOrientationAndJacobian(tk,&Jr,NULL);

  // Rearrange the spline jacobian matrices. Now Jpdot is the
  // jacobian of p wrt the spline coefficients stacked on top
  // of the jacobian of pdot wrt the spline coefficients.
  Jpdot.block(0, 0, 3, Jpdot.cols()) = Jp.block(3, 0, 3, Jp.cols());

  Matrix36 Jo;
  RP rp(p);
  omega = -C_w_b * rp.angularVelocityAndJacobian(pdot, &Jo);
  Jo = (-C_w_b * Jo).eval();
  //std::cout << "Jo:\n" << Jo << std::endl;
  if(J)
  {
    *J = Jo * Jpdot + skewSymmetric(omega) * Jr;
  }

  return omega;
}

// \omega_w_{b,w} (angular velocity of the body frame as seen from the world
// frame, expressed in the world frame)
template<class RP>
Vector3 BSplinePoseMinimal<RP>::angularVelocityAndJacobian(
    real_t tk,
    MatrixX* J,
    VectorXi* coefficient_indices) const
{

  Vector3 omega;
  Vector3 p;
  Vector3 pdot;
  MatrixX Jp;
  MatrixX Jpdot;
  Matrix61 v = evalDAndJacobian(tk, 0, &Jp, NULL);
  Matrix61 vdot = evalDAndJacobian(tk, 1, &Jpdot, coefficient_indices);
  p = v.tail<3>();
  pdot = vdot.tail<3>();

  // Rearrange the spline jacobian matrices. Now Jpdot is the
  // jacobian of p wrt the spline coefficients stacked on top
  // of the jacobian of pdot wrt the spline coefficients.
  Jpdot.block(0,0,3,Jpdot.cols()) = Jp.block(3,0,3,Jp.cols());

  //std::cout << "Jpdot\n" << Jpdot << std::endl;

  Matrix36 Jo;
  RP rp(p);
  omega = rp.angularVelocityAndJacobian(pdot, &Jo);

  //std::cout << "Jo:\n" << Jo << std::endl;
  if(J) {
    *J = Jo * Jpdot;
  }

  return omega;
}

template<class RP>
void BSplinePoseMinimal<RP>::initPoseSpline(
    real_t t0,
    real_t t1,
    const Matrix4& T_n_t0,
    const Matrix4& T_n_t1)
{
  VectorX v0 = transformationToCurveValue(T_n_t0);
  VectorX v1 = transformationToCurveValue(T_n_t1);

  initSpline(t0,t1,v0,v1);
}

template<class RP>
void BSplinePoseMinimal<RP>::addPoseSegment(real_t tk, const Matrix4& T_n_tk)
{
  VectorX vk = transformationToCurveValue(T_n_tk);

  addCurveSegment(tk, vk);
}

template<class RP>
void BSplinePoseMinimal<RP>::addPoseSegment2(
    real_t tk,
    const Matrix4& T_n_tk,
    real_t lambda)
{
  VectorX vk = transformationToCurveValue(T_n_tk);

  addCurveSegment2(tk, vk, lambda);
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::curveValueToTransformation(const VectorX& c) const
{
  CHECK_EQ(c.size(), 6) << "The curve value is an unexpected size!";
  Matrix4 T = Matrix4::Identity();
  T.topLeftCorner<3,3>() = RP(Vector3(c.tail<3>())).getRotationMatrix();
  T.topRightCorner<3,1>() = c.head<3>();

  return T;
}

template<class RP>
Matrix4 BSplinePoseMinimal<RP>::curveValueToTransformationAndJacobian(
    const VectorX& p, MatrixX * J) const
{
  CHECK_EQ(p.size(), 6) << "The curve value is an unexpected size!";
  Matrix4 T = Matrix4::Identity();
  Matrix3 S;
  RP rp(Vector3(p.tail<3>()));
  T.topLeftCorner<3,3>() = rp.getRotationMatrix();
  T.topRightCorner<3,1>() = p.head<3>();
  S = rp.toSMatrix();

  if(J)
  {
    *J = MatrixX::Identity(6,6);
    J->topRightCorner<3,3>() = -skewSymmetric(p.head<3>()) * S;
    J->bottomRightCorner<3,3>() = S;
  }

  return T;
}

template<class RP>
VectorX BSplinePoseMinimal<RP>::transformationToCurveValue(
    const Matrix4& T) const
{
  VectorX c(6);
  c.head<3>() = T.topRightCorner<3,1>();
  Matrix3 Tlc = T.topLeftCorner<3, 3>();
  RP rp(Tlc);
  c.tail<3>() = rp.getParameters();

  return c;
}

template<class RP>
void BSplinePoseMinimal<RP>::initPoseSpline2(
    const VectorX& times,
    const Eigen::Matrix<real_t,6,Eigen::Dynamic>& poses,
    int num_segments,
    real_t lambda)
{
  initSpline2(times, poses, num_segments, lambda);
}

template<class RP>
void BSplinePoseMinimal<RP>::initPoseSpline3(
    const VectorX& times,
    const Eigen::Matrix<real_t,6,Eigen::Dynamic>& poses,
    int num_segments,
    real_t lambda)
{
  initSpline3(times, poses, num_segments, lambda);
}

template<class RP>
void BSplinePoseMinimal<RP>::initPoseSplinePoses(
    const VectorX& times,
    const std::vector<Matrix4>& poses,
    int num_segments,
    real_t lambda)
{
  Eigen::Matrix<real_t, 6, Eigen::Dynamic> parameters;
  parameters.resize(6, poses.size());
  for (size_t i = 0; i < poses.size(); ++i)
  {
    parameters.col(i) = transformationToCurveValue(poses[i]);
  }

  initPoseSpline3(times, parameters, num_segments, lambda);
}

template<class RP>
void BSplinePoseMinimal<RP>::initPoseSplinePoses(
    const StampedTransformationVector& poses,
    int num_segments,
    real_t lambda)
{
  Eigen::Matrix<real_t, 6, Eigen::Dynamic> parameters;
  parameters.resize(6, poses.size());
  VectorX times(poses.size());
  for (size_t i = 0; i < poses.size(); ++i)
  {
    const StampedTransformation& pose = poses[i];
    parameters.col(i) =
        transformationToCurveValue(pose.second.getTransformationMatrix());
    times(i) = nanosecToSecTrunc(pose.first);
  }

  initPoseSpline3(times, parameters, num_segments, lambda);
}

// explicit specialization
template class BSplinePoseMinimal<ze::sm::RotationVector>;

} // namespace ze
