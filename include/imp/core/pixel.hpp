#ifndef IMP_PIXEL_HPP
#define IMP_PIXEL_HPP

#include <cstdint>
#include <cmath>

#ifdef WITH_CUDA
#  include<cuda_runtime_api.h>
#  define CUDA_HOST __host__
#  define CUDA_DEVICE  __device__
#else
#  define CUDA_HOST
#  define CUDA_DEVICE
#endif

namespace imp {

//------------------------------------------------------------------------------
template<typename _T>
union Pixel1
{
  using T = _T;

  struct
  {
    T x;
  };
  struct
  {
    T r;
  };
  T c[1];

  CUDA_HOST CUDA_DEVICE Pixel1() : x(0) { }
  CUDA_HOST CUDA_DEVICE Pixel1(T _x) : x(_x) { }
  CUDA_HOST CUDA_DEVICE ~Pixel1() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 1;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel2
{
  using T = _T;

  struct
  {
    T x,y;
  };
  struct
  {
    T r,g;
  };
  T c[2];

  CUDA_HOST CUDA_DEVICE Pixel2() : x(0), y(0) { }
  CUDA_HOST CUDA_DEVICE Pixel2(T _a) : x(_a), y(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel2(T _x, T _y) : x(_x), y(_y) { }
  CUDA_HOST CUDA_DEVICE ~Pixel2() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 2;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const Pixel2<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel3
{
  using T = _T;

  struct
  {
    T x,y,z;
  };
  struct
  {
    T r,g,b;
  };
  T c[3];

  CUDA_HOST CUDA_DEVICE Pixel3() : x(0), y(0), z(0) { }
  CUDA_HOST CUDA_DEVICE Pixel3(T _a) : x(_a), y(_a), z(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) { }
  CUDA_HOST CUDA_DEVICE ~Pixel3() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 3;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    c[2] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const Pixel3<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    c[2] *= rhs[2];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    c[2] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    c[2] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel4
{
  using T = _T;

  struct
  {
    T x,y,z,w;
  };
  struct
  {
    T r,g,b,a;
  };
  T c[4];

  CUDA_HOST CUDA_DEVICE Pixel4() : x(0), y(0), z(0), w(0) { }
  CUDA_HOST CUDA_DEVICE Pixel4(T _a) : x(_a), y(_a), z(_a), w(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel4(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w) { }
  CUDA_HOST CUDA_DEVICE ~Pixel4() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 4;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    c[2] *= rhs[0];
    c[3] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const Pixel4<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    c[2] *= rhs[2];
    c[3] *= rhs[3];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    c[2] *= rhs;
    c[3] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    c[2] /= rhs;
    c[3] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
// convenience typedefs
typedef Pixel1<std::uint8_t> Pixel8uC1;
typedef Pixel2<std::uint8_t> Pixel8uC2;
typedef Pixel3<std::uint8_t> Pixel8uC3;
typedef Pixel4<std::uint8_t> Pixel8uC4;

typedef Pixel1<std::uint16_t> Pixel16uC1;
typedef Pixel2<std::uint16_t> Pixel16uC2;
typedef Pixel3<std::uint16_t> Pixel16uC3;
typedef Pixel4<std::uint16_t> Pixel16uC4;

typedef Pixel1<std::int32_t> Pixel32sC1;
typedef Pixel2<std::int32_t> Pixel32sC2;
typedef Pixel3<std::int32_t> Pixel32sC3;
typedef Pixel4<std::int32_t> Pixel32sC4;

typedef Pixel1<std::uint32_t> Pixel32uC1;
typedef Pixel2<std::uint32_t> Pixel32uC2;
typedef Pixel3<std::uint32_t> Pixel32uC3;
typedef Pixel4<std::uint32_t> Pixel32uC4;

typedef Pixel1<float> Pixel32fC1;
typedef Pixel2<float> Pixel32fC2;
typedef Pixel3<float> Pixel32fC3;
typedef Pixel4<float> Pixel32fC4;


// vector types (same as pixel)
template<typename T> using Vec1 = Pixel1<T>;
template<typename T> using Vec2 = Pixel2<T>;
template<typename T> using Vec3 = Pixel3<T>;
template<typename T> using Vec4 = Pixel4<T>;

using Vec8uC1 = Vec1<std::uint8_t>;
using Vec8uC2 = Vec2<std::uint8_t>;
using Vec8uC3 = Vec3<std::uint8_t>;
using Vec8uC4 = Vec4<std::uint8_t>;

using Vec16uC1 = Vec1<std::uint16_t>;
using Vec16uC2 = Vec2<std::uint16_t>;
using Vec16uC3 = Vec3<std::uint16_t>;
using Vec16uC4 = Vec4<std::uint16_t>;

using Vec32sC1 = Vec1<std::int32_t>;
using Vec32sC2 = Vec2<std::int32_t>;
using Vec32sC3 = Vec3<std::int32_t>;
using Vec32sC4 = Vec4<std::int32_t>;

using Vec32uC1 = Vec1<std::uint32_t>;
using Vec32uC2 = Vec2<std::uint32_t>;
using Vec32uC3 = Vec3<std::uint32_t>;
using Vec32uC4 = Vec4<std::uint32_t>;

using Vec32fC1 = Vec1<float>;
using Vec32fC2 = Vec2<float>;
using Vec32fC3 = Vec3<float>;
using Vec32fC4 = Vec4<float>;



//------------------------------------------------------------------------------
// comparison operators
template<typename T>
inline bool operator==(const Pixel1<T>& lhs, const Pixel1<T>& rhs)
{
  return (lhs.x == rhs.x);
}

//------------------------------------------------------------------------------
// dot product

template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec2<T> a, Vec2<T> b)
{
  return a.x * b.x + a.y * b.y;
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec3<T> a, Vec3<T> b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec4<T> a, Vec4<T> b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

//------------------------------------------------------------------------------
// length
template<typename T>
inline CUDA_HOST CUDA_DEVICE float length(T v)
{
  return std::sqrt((float)dot(v,v));
}

//------------------------------------------------------------------------------
//normalize

template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC2 normalize(Vec2<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC2(v.x*inv_len, v.y*inv_len);
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC3 normalize(Vec3<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC3(v.x*inv_len, v.y*inv_len, v.z*inv_len);
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC4 normalize(Vec4<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC4(v.x*inv_len, v.y*inv_len, v.z*inv_len, v.w*inv_len);
}


} // namespace imp

#undef CUDA_HOST
#undef CUDA_DEVICE


#endif // IMP_PIXEL_HPP

