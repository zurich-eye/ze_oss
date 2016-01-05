#ifndef IMP_CU_SE3_CUH
#define IMP_CU_SE3_CUH

#include <ostream>
//#include <imp/cuda_toolkit/helper_math.h>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_matrix.cuh>

namespace imp
{
namespace cu
{

template<typename Type>
class SE3
{
public:
  __host__ __device__ __forceinline__
  SE3()
  {
    data_(0, 0) = 1;
    data_(0, 1) = 0;
    data_(0, 2) = 0;
    data_(1, 0) = 0;
    data_(1, 1) = 1;
    data_(1, 2) = 0;
    data_(2, 0) = 0;
    data_(2, 1) = 0;
    data_(2, 2) = 1;

    data_(0, 3) = 0;
    data_(1, 3) = 0;
    data_(2, 3) = 0 ;
  }

  /// Constructor from a normalized quaternion and a translation vector
  __host__ __device__ __forceinline__
  SE3(Type qw, Type qx, Type qy, Type qz, Type tx, Type ty, Type tz)
  {
    const Type x  = 2*qx;
    const Type y  = 2*qy;
    const Type z  = 2*qz;
    const Type wx = x*qw;
    const Type wy = y*qw;
    const Type wz = z*qw;
    const Type xx = x*qx;
    const Type xy = y*qx;
    const Type xz = z*qx;
    const Type yy = y*qy;
    const Type yz = z*qy;
    const Type zz = z*qz;

    data_(0, 0) = 1-(yy+zz);
    data_(0, 1) = xy-wz;
    data_(0, 2) = xz+wy;
    data_(1, 0) = xy+wz;
    data_(1, 1) = 1-(xx+zz);
    data_(1, 2) = yz-wx;
    data_(2, 0) = xz-wy;
    data_(2, 1) = yz+wx;
    data_(2, 2) = 1-(xx+yy);

    data_(0, 3) = tx;
    data_(1, 3) = ty;
    data_(2, 3) = tz;
  }

  /// Construct from C arrays
  /// r is rotation matrix row major
  /// t is the translation vector (x y z)
  __host__ __device__ __forceinline__
  SE3(Type * r, Type * t)
  {
    data_[0]=r[0]; data_[1]=r[1]; data_[2] =r[2]; data_[3] =t[0];
    data_[4]=r[3]; data_[5]=r[4]; data_[6] =r[5]; data_[7] =t[1];
    data_[8]=r[6]; data_[9]=r[7]; data_[10]=r[8]; data_[11]=t[2];
  }

  /// Construct from C array of a 4x4 transformation matrix
  /// m is a 4x4 transformation matrix (with [0 0 0 1] in the last row)
  __host__ __device__ __forceinline__
  SE3(Type *m)
  {
    data_[0]=m[0]; data_[1]=m[1]; data_[2] =m[2]; data_[3] =m[3];
    data_[4]=m[4]; data_[5]=m[5]; data_[6] =m[6]; data_[7] =m[7];
    data_[8]=m[8]; data_[9]=m[9]; data_[10]=m[10]; data_[11]=m[11];
  }

  __host__ __device__ __forceinline__
  SE3<Type> inv() const
  {
    SE3<Type> result;
    result.data_[0]  = data_[0];
    result.data_[1]  = data_[4];
    result.data_[2]  = data_[8];
    result.data_[4]  = data_[1];
    result.data_[5]  = data_[5];
    result.data_[6]  = data_[9];
    result.data_[8]  = data_[2];
    result.data_[9]  = data_[6];
    result.data_[10] = data_[10];
    result.data_[3]  = -data_[0]*data_[3] -data_[4]*data_[7] -data_[8] *data_[11];
    result.data_[7]  = -data_[1]*data_[3] -data_[5]*data_[7] -data_[9] *data_[11];
    result.data_[11] = -data_[2]*data_[3] -data_[6]*data_[7] -data_[10]*data_[11];
    return result;
  }

  __host__ __device__ __forceinline__
  Type operator()(int r, int c) const
  {
    return data_(r, c);
  }

  __host__ __device__ __forceinline__
  Type& operator()(int r, int c)
  {
    return data_(r, c);
  }

  __host__ __device__ __forceinline__
  Type& operator[](int idx)
  {
    return data_[idx];
  }

  __host__ __device__ __forceinline__
  const Type& operator[](int idx) const
  {
    return data_[idx];
  }

  __host__ __device__ __forceinline__
  Vec32fC3 rotate(const Vec32fC3& p) const
  {
    return Vec32fC3(data_(0,0)*p.x + data_(0,1)*p.y + data_(0,2)*p.z,
                    data_(1,0)*p.x + data_(1,1)*p.y + data_(1,2)*p.z,
                    data_(2,0)*p.x + data_(2,1)*p.y + data_(2,2)*p.z);
  }
  __host__ __device__ __forceinline__
  float3 rotate(const float3& p) const
  {
    return make_float3(data_(0,0)*p.x + data_(0,1)*p.y + data_(0,2)*p.z,
                       data_(1,0)*p.x + data_(1,1)*p.y + data_(1,2)*p.z,
                       data_(2,0)*p.x + data_(2,1)*p.y + data_(2,2)*p.z);
  }

  __host__ __device__ __forceinline__
  Vec32fC3 translate(const Vec32fC3& p) const
  {
    return Vec32fC3(p.x + data_(0,3),
                    p.y + data_(1,3),
                    p.z + data_(2,3));
  }

  __host__ __device__ __forceinline__
  float3 translate(const float3& p) const
  {
    return make_float3(p.x + data_(0,3),
                       p.y + data_(1,3),
                       p.z + data_(2,3));
  }

  __host__ __device__ __forceinline__
  Vec32fC3 getTranslation() const
  {
    return Vec32fC3(data_(0,3),
                    data_(1,3),
                    data_(2,3));
  }

private:
  imp::cu::Matrix<Type, 3, 4> data_;
};

template<typename Type>
__host__ __device__ __forceinline__
SE3<Type> operator*(const SE3<Type>& lhs, const SE3<Type>& rhs)
{
  SE3<Type> result;
  result[0]  = lhs[0]*rhs[0] + lhs[1]*rhs[4] + lhs[2]*rhs[8];
  result[1]  = lhs[0]*rhs[1] + lhs[1]*rhs[5] + lhs[2]*rhs[9];
  result[2]  = lhs[0]*rhs[2] + lhs[1]*rhs[6] + lhs[2]*rhs[10];
  result[3]  = lhs[3] + lhs[0]*rhs[3] + lhs[1]*rhs[7] + lhs[2]*rhs[11];
  result[4]  = lhs[4]*rhs[0] + lhs[5]*rhs[4] + lhs[6]*rhs[8];
  result[5]  = lhs[4]*rhs[1] + lhs[5]*rhs[5] + lhs[6]*rhs[9];
  result[6]  = lhs[4]*rhs[2] + lhs[5]*rhs[6] + lhs[6]*rhs[10];
  result[7]  = lhs[7] + lhs[4]*rhs[3] + lhs[5]*rhs[7] + lhs[6]*rhs[11];
  result[8]  = lhs[8]*rhs[0] + lhs[9]*rhs[4] + lhs[10]*rhs[8];
  result[9]  = lhs[8]*rhs[1] + lhs[9]*rhs[5] + lhs[10]*rhs[9];
  result[10] = lhs[8]*rhs[2] + lhs[9]*rhs[6] + lhs[10]*rhs[10];
  result[11] = lhs[11] + lhs[8]*rhs[3] + lhs[9]*rhs[7] + lhs[10]*rhs[11];
  return result;
}

__host__ __device__ __forceinline__
Vec32fC3 operator*(const SE3<float>& se3, const Vec32fC3& p)
{
  return se3.translate(se3.rotate(p));
}

__host__ __device__ __forceinline__
float3 operator*(const SE3<float>& se3, const float3& p)
{
  return se3.translate(se3.rotate(p));
}

template<typename T>
inline std::ostream& operator<<(std::ostream &os, const cu::SE3<T>& rhs)
{
  for (int r=0; r<3; ++r)
  {
    for (int c=0; c<4; ++c)
    {
      os << rhs(r,c) << ", ";
    }
    os << "; ";
  }
  return os;
}

} // namespace cu
} // namespace imp

#endif // IMP_CU_SE3_CUH
