#ifndef IMP_CU_SETVALUE_CUH
#define IMP_CU_SETVALUE_CUH

//#include <stdio.h>

#include <cuda_runtime_api.h>

#if 0
#include <imp/cu_core/cu_gpu_data.cuh>
#endif

namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
/**
 * @brief
 */
#if 0
template<typename Pixel>
__global__ void k_setValue(GpuData2D<Pixel>* dst, const Pixel value)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  // account for roi
  x+=dst->roi.x();
  y+=dst->roi.y();

  if (dst->inRoi(x,y))
  {
    dst->data[y*dst->stride+x] = value;
  }
}
#endif


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_setValue(
    Pixel* d_dst, std::uint32_t offset, std::uint32_t roi_length, const Pixel value)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  if (x<roi_length)
  {
    d_dst[x+offset] = value;
  }
}


//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_setValue(Pixel* d_dst, size_t stride, const Pixel value,
                           size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x>=0 && y>=0 && x<width && y<height)
  {
    d_dst[y*stride+x] = value;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, typename T>
__global__ void k_pixelWiseMul(Pixel* d_dst, size_t stride, const T rhs,
                               size_t width, size_t height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x>=0 && y>=0 && x<width && y<height)
  {
    d_dst[y*stride+x] = d_dst[y*stride+x]*rhs;
  }
}


} // namespace cu
} // namespace imp

#endif // IMP_CU_SETVALUE_CUH
