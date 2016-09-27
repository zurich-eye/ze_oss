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
#pragma once

#include <cuda_runtime.h>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
# include <Eigen/Dense>
#pragma GCC diagnostic pop
#include <memory>
#include <ostream>

#include <imp/core/pixel.hpp>
#include <ze/common/matrix.hpp>

namespace ze {
namespace cu {

//------------------------------------------------------------------------------
template<typename _Type, uint32_t _rows, uint32_t _cols>
class Matrix
{
  using Type = _Type;

public:
  typedef std::shared_ptr<Matrix> Ptr;

  __host__ __device__
  Matrix() = default;

  __host__ __device__
  ~Matrix() = default;

  __host__
  Matrix(const ze::Matrix3& from)
  {
    // copy element by element
    for (uint32_t row = 0u; row < _rows; ++row)
    {
      for (uint32_t col = 0u; col < _cols; ++col)
      {
        data_[row*cols_ + col] = from(row, col);
      }
    }
  }

  __host__ __device__ __forceinline__
  uint32_t rows() const { return rows_; }

  __host__ __device__ __forceinline__
  uint32_t cols() const { return cols_; }

  /** Data access operator given a \a row and a \a col
   * @return unchangable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  const Type& operator()(uint32_t row, uint32_t col) const
  {
    return data_[row*cols_ + col];
  }

  /** Data access operator given a \a row and a \a col
   * @return changable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  Type& operator()(uint32_t row, uint32_t col)
  {
    return data_[row*cols_ + col];
  }

  /** Data access operator given an \a index
   * @return unchangable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  const Type& operator[](uint32_t ind) const
  {
    return data_[ind];
  }

  /** Data access operator given an \a index
   * @return changable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  Type & operator[](uint32_t ind)
  {
    return data_[ind];
  }

  template<uint32_t block_rows,uint32_t block_cols>
  __host__ __forceinline__
  Matrix<Type, block_rows, block_cols> block(uint32_t top_left_row, uint32_t top_left_col) const
  {
    Matrix<Type, block_rows, block_cols> out;
    uint32_t data_offset = top_left_row * cols_ + top_left_col;
    for(uint32_t row = 0u; row < block_rows; ++row)
    {
      memcpy(&out[row * block_cols],
             &data_[data_offset+row*cols_],
             block_cols * sizeof(Type));
    }
    return out;
  }

#if 0
  template<typename TypeFrom>
  __host__ inline Matrix(const Eigen::Matrix<TypeFrom,R,C>& mat)
  {
    for (uint32_t row = 0u; row < R; ++row)
    {
      for (uint32_t col = 0u; col < C; ++col)
      {
        data[row * C + col] = (Type)mat(row,col);
      }
    }
  }
#endif

private:
  Type data_[_rows * _cols];
  uint32_t rows_ = _rows;
  uint32_t cols_ = _cols;
};

//------------------------------------------------------------------------------
// convenience typedefs
using Matrix3f = Matrix<float,3,3>;
using Vector3f = Matrix<float,1,3>;

//==============================================================================


//------------------------------------------------------------------------------
template<typename Type, uint32_t _rows, uint32_t CR, uint32_t _cols>
__host__ __device__ __forceinline__
Matrix<Type, _rows, _cols> operator*(const Matrix<Type, _rows, CR> & lhs,
                                     const Matrix<Type, CR, _cols> & rhs)
{
  Matrix<Type, _rows, _cols> result;
  for (uint32_t row = 0u; row < _rows; ++row)
  {
    for (uint32_t col = 0u; col < _cols; ++col)
    {
      result(row, col) = 0;
      for (uint32_t i = 0u; i < CR; ++i)
      {
        result(row, col) += lhs(row,i) * rhs(i,col);
      }
    }
  }
  return result;
}

//------------------------------------------------------------------------------
template<typename Type>
__host__ __device__ __forceinline__
Matrix<Type, 2, 2> invert(const Matrix<Type, 2, 2> & in)
{
  Matrix<Type, 2, 2> out;
  float det = in[0]*in[3] - in[1]*in[2];
  out[0] =  in[3] / det;
  out[1] = -in[1] / det;
  out[2] = -in[2] / det;
  out[3] =  in[0] / det;
  return out;
}


//------------------------------------------------------------------------------
// matrix vector multiplication
__host__ __device__ __forceinline__
float3 operator*(const Matrix3f& mat, const float3& v)
{
  return make_float3(
        mat(0,0)*v.x + mat(0,1)*v.y + mat(0,2)*v.z,
        mat(1,0)*v.x + mat(1,1)*v.y + mat(1,2)*v.z,
        mat(2,0)*v.x + mat(2,1)*v.y + mat(2,2)*v.z
        );
}

//------------------------------------------------------------------------------
// matrix vector multiplication
__host__ __device__ __forceinline__
Vec32fC3 operator*(const Matrix3f& mat, const Vec32fC3& v)
{
  return Vec32fC3(
        mat(0,0)*v.x + mat(0,1)*v.y + mat(0,2)*v.z,
        mat(1,0)*v.x + mat(1,1)*v.y + mat(1,2)*v.z,
        mat(2,0)*v.x + mat(2,1)*v.y + mat(2,2)*v.z
        );
}

//------------------------------------------------------------------------------
// transformation matrix three-vector multiplication
__host__ __device__ __forceinline__
float3 transform(const Matrix<float,3,4>& T, const float3& v)
{
  return make_float3(
        T(0,0)*v.x + T(0,1)*v.y + T(0,2)*v.z + T(0,3),
        T(1,0)*v.x + T(1,1)*v.y + T(1,2)*v.z + T(1,3),
        T(2,0)*v.x + T(2,1)*v.y + T(2,2)*v.z + T(2,3)
        );
}

//------------------------------------------------------------------------------
template<typename T, uint32_t rows, uint32_t cols>
__host__
inline std::ostream& operator<<(std::ostream &os,
                                const cu::Matrix<T, rows, cols>& m)
{
  os << "[";
  for (uint32_t r=0u; r<rows; ++r)
  {
    for (uint32_t c=0u; c<cols; ++c)
    {
      os << m(r,c);
      if (c<cols-1)
      {
        os << ",";
      }
    }
    os << "; ";
  }
  os << "]";
  return os;
}

} // namespace cu
} // namespace ze
