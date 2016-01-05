#ifndef IMP_CU_UTILS_CUH
#define IMP_CU_UTILS_CUH

#include <cuda_runtime_api.h>
#include <cstdint>
#ifndef __CUDA__ARCH__
#include <cmath>
#endif
#include <type_traits>

#include <imp/core/size.hpp>
#include <imp/core/roi.hpp>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_exception.hpp>

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
/** Integer division rounding up to next higher integer
 * @param a Numerator
 * @param b Denominator
 * @return a / b rounded up
 */
__host__ __device__ __forceinline__
std::uint32_t divUp(std::uint32_t a, std::uint32_t b)
{
  return (a % b != 0) ? (a / b + 1) : (a / b);
}

template<typename T>
__host__ __device__ __forceinline__
bool isfinite(const T& val)
{
  return true;
//#ifdef __CUDA__ARCH__
//  return ::isfinite(val);
//#else
//  return std::isfinite(val);
//#endif
}

template<typename T>
__host__ __device__ __forceinline__
typename std::enable_if<std::is_integral<T>::value ||std::is_floating_point<T>::value, T>::type
min(const T& a, const T& b, bool check_inf_or_nan=true)
{
  if (imp::cu::isfinite(a) && imp::cu::isfinite(b))
    return a<b ? a : b;
  else if (imp::cu::isfinite(a))
    return a;
  else
    return b;

//    if (check_inf_or_nan)
//    {
//      if (isnan(a) || isinf(a))
//        return b;
//      if (isnan(b) || isinf(b))
//        return a;
//    }
}

//template<typename T>
//__host__ __device__ __forceinline__
//typename std::enable_if<std::is_floating_point<T>::value, T>::type
//min(const T& a, const T& b, bool check_inf_or_nan=true)
//{
//    if (check_inf_or_nan)
//    {
//      if (std::isnan(a) || std::isinf(a))
//        return b;
//      if (std::isnan(b) || std::isinf(b))
//        return a;
//    }
//  return a<b ? a : b;
//}

template<typename T>
__host__ __device__ __forceinline__
Pixel1<T> min(const Pixel1<T>& a, const Pixel1<T>& b)
{
  Pixel1<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]<b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel2<T> min(const Pixel2<T>& a, const Pixel2<T>& b)
{
  Pixel2<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]<b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel3<T> min(const Pixel3<T>& a, const Pixel3<T>& b)
{
  Pixel3<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]<b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel4<T> min(const Pixel4<T>& a, const Pixel4<T>& b)
{
  Pixel4<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]<b[i]) ? a[i] : b[i];
  return result;
}

//------------------------------------------------------------------------------
template<typename T>
__host__ __device__ __forceinline__
typename std::enable_if<std::is_integral<T>::value ||std::is_floating_point<T>::value, T>::type
max(const T& a, const T& b, bool check_inf_or_nan=true)
{
  if (imp::cu::isfinite(a) && imp::cu::isfinite(b))
    return a>b ? a : b;
  else if (imp::cu::isfinite(a))
    return a;
  else
    return b;

}

//template<typename T>
//__host__ __device__ __forceinline__
//typename std::enable_if<std::is_floating_point<T>::value, T>::type
//max(const T& a, const T& b, bool check_inf_or_nan=true)
//{
//    if (check_inf_or_nan)
//    {
//      if (std::isnan(a) || std::isinf(a))
//        return b;
//      if (std::isnan(b) || std::isinf(b))
//        return a;
//    }
//  return a>b ? a : b;
//}

template<typename T>
__host__ __device__ __forceinline__
Pixel1<T> max(const Pixel1<T>& a, const Pixel1<T>& b)
{
  Pixel1<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]>b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel2<T> max(const Pixel2<T>& a, const Pixel2<T>& b)
{
  Pixel2<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]>b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel3<T> max(const Pixel3<T>& a, const Pixel3<T>& b)
{
  Pixel3<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]>b[i]) ? a[i] : b[i];
  return result;
}
template<typename T>
__host__ __device__ __forceinline__
Pixel4<T> max(const Pixel4<T>& a, const Pixel4<T>& b)
{
  Pixel4<T> result;
  for (auto i=0; i<a.numDims(); ++i)
    result[i] = (a[i]>b[i]) ? a[i] : b[i];
  return result;
}

template<typename T>
__host__ __device__ __forceinline__
typename std::enable_if<!std::is_floating_point<T>::value && !std::is_integral<T>::value, T>::type
min(const T& a, const T& b, bool check_inf_or_nan=true)
{
  return a<b ? a : b;
}

/// @todo (MWE) rvalue?
template<typename T>
__host__ __device__ __forceinline__
T sqr(const T& a) {return a*a;}


/** Fragmentation for gpu / cuda grid blocks
 */
template <std::uint16_t block_size_x=32,
          std::uint16_t block_size_y=32,
          std::uint16_t block_size_z=1>
struct Fragmentation
{
  //  imp::Size2u size;
  //  imp::Roi2u roi;
  dim3 dimBlock = dim3(block_size_x, block_size_y, block_size_z);
  dim3 dimGrid;


  Fragmentation() = delete;

  Fragmentation(size_t length)
    : dimGrid(divUp(length, dimBlock.x), dimBlock.x, dimBlock.y)
  {
  }

  Fragmentation(imp::Size2u sz)
    : dimGrid(divUp(sz.width(), dimBlock.x), divUp(sz.height(), dimBlock.y))
  {
  }

  Fragmentation(imp::Roi2u roi)
    : dimGrid(divUp(roi.width(), dimBlock.x), divUp(roi.height(), dimBlock.y))
  {
  }
  Fragmentation(std::uint32_t width, std::uint32_t height)
    : dimGrid(divUp(width, dimBlock.x), divUp(height, dimBlock.y))
  {
  }

  Fragmentation(dim3 _dimGrid, dim3 _dimBlock)
    : dimGrid(_dimGrid)
    , dimBlock(_dimBlock)
  {
  }
};

//------------------------------------------------------------------------------
template <std::uint16_t block_size_x=16,
          std::uint16_t block_size_y=16,
          std::uint16_t block_size_z=1>
inline std::ostream& operator<<(
    std::ostream &os,
    const  Fragmentation<block_size_x, block_size_y, block_size_z>& frag)
{
  os << "GPU Fragmentation: block: "
     << frag.dimBlock.x << "," << frag.dimBlock.y << "," << frag.dimBlock.z
     << "; grid: " << frag.dimGrid.x << "," << frag.dimGrid.y << "," << frag.dimGrid.z
     << ";";
  return os;
}

//##############################################################################

/** Check for CUDA error */
static inline void checkCudaErrorState(const char* file, const char* function,
                                       const int line)
{
  cudaDeviceSynchronize();
  cudaError_t err = cudaGetLastError();
  if( err != ::cudaSuccess )
    throw imp::cu::Exception("error state check", err, file, function, line);
}

/** Macro for checking on cuda errors
 * @note This check is only enabled when the compile time flag is set
 * @todo (MWE) we should enable this whenever we compile in debug mode
 */
#ifdef THROW_ON_CUDA_ERROR
#  define IMP_CUDA_CHECK() imp::cu::checkCudaErrorState(__FILE__, __FUNCTION__, __LINE__)
#else
#  define IMP_CUDA_CHECK() cudaDeviceSynchronize()
#endif

static inline float getTotalGPUMemory()
{
  size_t total = 0;
  size_t free = 0;
  cudaMemGetInfo(&free, &total);
  return total/(1024.0f*1024.0f);   // return value in Megabytes
}

static inline float getFreeGPUMemory()
{
  size_t total = 0;
  size_t free = 0;
  cudaMemGetInfo(&free, &total);
  return free/(1024.0f*1024.0f);   // return value in Megabytes
}

static inline void printGPUMemoryUsage()
{
  float total = imp::cu::getTotalGPUMemory();
  float free = imp::cu::getFreeGPUMemory();

  printf("GPU memory usage\n");
  printf("----------------\n");
  printf("   Total memory: %.2f MiB\n", total);
  printf("   Used memory:  %.2f MiB\n", total-free);
  printf("   Free memory:  %.2f MiB\n", free);
}

/** @} */ // end of Error Handling
/** @} */ // end of Cuda Utilities


} // namespace cu
} // namespace imp

#endif // IMP_CU_UTILS_CUH

