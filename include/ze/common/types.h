#pragma once

#include <cstdint>
#include <Eigen/Core>

namespace ze {

//------------------------------------------------------------------------------
// Scalars and fp precision.
using size_t    = std::size_t;
using int64_t   = std::int64_t;   
using uint8_t   = std::uint8_t;
using uint32_t  = std::uint32_t;
using uint64_t  = std::uint64_t;
using FloatType = double;

//------------------------------------------------------------------------------
// Typedefs of commonly used Eigen matrices and vectors.

// MatrixMN, MatrixN = MatrixNN, I_NxN, and Z_NxN, for M,N=1..9.
#define ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(SIZE, SUFFIX)         \
  using Matrix##SUFFIX = Eigen::Matrix<FloatType, SIZE, SIZE>; \
  using Matrix1##SUFFIX = Eigen::Matrix<FloatType, 1, SIZE>;   \
  using Matrix2##SUFFIX = Eigen::Matrix<FloatType, 2, SIZE>;   \
  using Matrix3##SUFFIX = Eigen::Matrix<FloatType, 3, SIZE>;   \
  using Matrix4##SUFFIX = Eigen::Matrix<FloatType, 4, SIZE>;   \
  using Matrix5##SUFFIX = Eigen::Matrix<FloatType, 5, SIZE>;   \
  using Matrix6##SUFFIX = Eigen::Matrix<FloatType, 6, SIZE>;   \
  using Matrix7##SUFFIX = Eigen::Matrix<FloatType, 7, SIZE>;   \
  using Matrix8##SUFFIX = Eigen::Matrix<FloatType, 8, SIZE>;   \
  using Matrix9##SUFFIX = Eigen::Matrix<FloatType, 9, SIZE>;   \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::IdentityReturnType I_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Identity(); \
  static const Eigen::MatrixBase<Matrix##SUFFIX>::ConstantReturnType Z_##SUFFIX##x##SUFFIX = Matrix##SUFFIX::Zero()

ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(1,1);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(2,2);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(3,3);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(4,4);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(5,5);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(6,6);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(7,7);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(8,8);
ZE_MAKE_EIGEN_MATRIX_TYPEDEFS(9,9);

// Typedef arbitary length vector.
using VectorX  = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

// Commonly used fixed size vectors.
using Vector1 = Eigen::Matrix<FloatType, 1, 1>;
using Vector2 = Eigen::Matrix<FloatType, 2, 1>;
using Vector3 = Eigen::Matrix<FloatType, 3, 1>;
using Vector4 = Eigen::Matrix<FloatType, 4, 1>;
using Vector5 = Eigen::Matrix<FloatType, 5, 1>;
using Vector6 = Eigen::Matrix<FloatType, 6, 1>;
using Vector7 = Eigen::Matrix<FloatType, 7, 1>;
using Vector8 = Eigen::Matrix<FloatType, 8, 1>;
using Vector9 = Eigen::Matrix<FloatType, 9, 1>;
using Vector2i = Eigen::Vector2i;

//------------------------------------------------------------------------------
// Feature containers.
using Keypoint = Eigen::Matrix<FloatType, 2, 1>;
using Bearing = Eigen::Matrix<FloatType, 3, 1>;
using Position = Eigen::Matrix<FloatType, 3, 1>;
using Gradient = Eigen::Matrix<FloatType, 2, 1>;
using Keypoints = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;
using Bearings = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Positions = Eigen::Matrix<FloatType, 3, Eigen::Dynamic, Eigen::ColMajor>;
using Gradients = Eigen::Matrix<FloatType, 2, Eigen::Dynamic, Eigen::ColMajor>;

//------------------------------------------------------------------------------
// Inertial containers.
using ImuStamps = VectorX;
using ImuAccGyr = Eigen::Matrix<FloatType, 6, Eigen::Dynamic, Eigen::ColMajor>;



} // namespace ze
