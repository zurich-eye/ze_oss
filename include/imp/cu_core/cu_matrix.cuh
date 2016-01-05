#ifndef IMP_CU_MATRIX_CUH
#define IMP_CU_MATRIX_CUH

#include <memory>
#include <ostream>
#include <cuda_runtime.h>
#include <imp/core/pixel.hpp>
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
# include <Eigen/Dense>
#pragma GCC diagnostic pop

namespace imp{
namespace cu{

//------------------------------------------------------------------------------
template<typename _Type, size_t _rows, size_t _cols>
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
  Matrix(Eigen::Matrix<Type,_rows,_cols,Eigen::RowMajor> from)
  {
    // check if memory is unpadded
    if(from.innerStride() == 1)
    {
      if(from.outerStride() == _cols)
      {
        // copy whole memory block
        memcpy(data_,from.data(),_rows*_cols*sizeof(Type));
      }
      else
      {
        // copy line by line
        for(int row = 0; row < _rows; ++row)
        {
          memcpy(&data_[row*_cols],&(from.data()[row*from.outerStride()]),_cols*sizeof(Type));
        }
      }
    }
    else
    {
      // copy element by element
      for(int row = 0; row < _rows; ++row)
      {
        for(int col = 0; col < _cols; ++col)
        {
          data_[row*cols_ + col] = from(row,col);
        }
      }
    }
  }

  //  // copy and asignment operator
  //  __host__ __device__
  //  Matrix(const Matrix& other)
  //    : f_(other.f())
  //    , c_(other.c())
  //  {
  //  }
  //  __host__ __device__
  //  Matrix& operator=(const Matrix& other)
  //  {
  //    if  (this != &other)
  //    {
  //      f_ = other.f();
  //      c_ = other.c();
  //    }
  //    return *this;
  //  }


  __host__ __device__ __forceinline__
  size_t rows() const { return rows_; }

  __host__ __device__ __forceinline__
  size_t cols() const { return cols_; }

  /** Data access operator given a \a row and a \a col
   * @return unchangable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  const Type& operator()(int row, int col) const
  {
    return data_[row*cols_ + col];
  }

  /** Data access operator given a \a row and a \a col
   * @return changable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  Type& operator()(int row, int col)
  {
    return data_[row*cols_ + col];
  }

  /** Data access operator given an \a index
   * @return unchangable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  const Type& operator[](int ind) const
  {
    return data_[ind];
  }

  /** Data access operator given an \a index
   * @return changable value at \a (row,col)
   */
  __host__ __device__ __forceinline__
  Type & operator[](int ind)
  {
    return data_[ind];
  }

  template<size_t block_rows,size_t block_cols>
  __host__ __forceinline__
  Matrix<Type,block_rows,block_cols> block(size_t top_left_row, size_t top_left_col) const
  {
    Matrix<Type,block_rows,block_cols> out;
    size_t data_offset = top_left_row*cols_ + top_left_col;
    for(size_t row = 0; row < block_rows; ++row)
    {
      memcpy(&out[row*block_cols],&data_[data_offset+row*cols_],block_cols*sizeof(Type));
    }
    return out;
  }

#if 0
  template<typename TypeFrom>
  __host__ inline Matrix(const Eigen::Matrix<TypeFrom,R,C>& mat)
  {
    for (size_t row=0; row<R; ++row)
    {
      for (size_t col=0; col<C; ++col)
      {
        data[row*C+col] = (Type)mat(row,col);
      }
    }
  }
#endif

private:
  Type data_[_rows*_cols];
  size_t rows_ = _rows;
  size_t cols_ = _cols;
};

//------------------------------------------------------------------------------
// convenience typedefs
using Matrix3f = Matrix<float,3,3>;
using Vector3f = Matrix<float,1,3>;

//==============================================================================


//------------------------------------------------------------------------------
template<typename Type, size_t _rows, size_t CR, size_t _cols>
__host__ __device__ __forceinline__
Matrix<Type, _rows, _cols> operator*(const Matrix<Type, _rows, CR> & lhs,
                                     const Matrix<Type, CR, _cols> & rhs)
{
  Matrix<Type, _rows, _cols> result;
  for(size_t row=0; row<_rows; ++row)
  {
    for(size_t col=0; col<_cols; ++col)
    {
      result(row, col) = 0;
      for(size_t i=0; i<CR; ++i)
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
template<typename T, size_t rows, size_t cols>
__host__
inline std::ostream& operator<<(std::ostream &os,
                                const cu::Matrix<T, rows, cols>& m)
{
  os << "[";
  for (int r=0; r<rows; ++r)
  {
    for (int c=0; c<cols; ++c)
    {
      os << m(r,c);
      if (c<cols-1)
        os << ",";
    }
    os << "; ";
  }
  os << "]";
  return os;
}

}
}

#endif // IMP_CU_MATRIX_CUH

