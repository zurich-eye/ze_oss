#ifndef IMP_CU_BILATERAL_IMPL_CU
#define IMP_CU_BILATERAL_IMPL_CU

#include <imp/cu_imgproc/cu_image_filter.cuh>

#include <cstdint>
#include <cuda_runtime.h>

#include <imp/core/types.hpp>
#include <imp/core/roi.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>



namespace imp {
namespace cu {

// ----------------------------------------------------------------------------
// kernel: bilateral filter kernel C1
__global__ void cuFilterBilateralKernel_32f_C1(const float* src, float* dst,
                                               const float* prior,
                                               const float sigma_spatial, const float sigma_range,
                                               const int radius, const size_t stride,
                                               const int xoff, const int yoff,
                                               const int width, const int height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x + xoff;
  int y = blockIdx.y*blockDim.y + threadIdx.y + yoff;

  int c = y*stride+x;
  float p = prior[c];

  if(x<width && y<height)
  {
    float sum_g = 0.0f;
    float sum_val = 0.0f;

    for (int l=-radius; l<=radius; ++l)
    {
      for (int k=-radius; k<=radius; ++k)
      {
        int xx=x+k, yy=y+l;
        if(xx>=0 && yy>=0 && xx<width && yy<height)
        {
          int cc = yy*stride+xx;
          float g = expf(-((iu::sqr(x-xx)+iu::sqr(y-yy))/(2.0f*iu::sqr(sigma_spatial)))
                         -(iu::sqr(p-prior[cc])/(2.0f*iu::sqr(sigma_range))));
          sum_g += g;
          sum_val += g*src[cc];
        }
      }
    }

    dst[c] = sum_val / IUMAX(1e-6f, sum_g);
  }

}

// ----------------------------------------------------------------------------
// kernel: bilateral filter kernel C1 with C4 prior
__global__ void cuFilterBilateralKernel_32f_C1C4(const float* src, float* dst,
                                                 const float4* prior,
                                                 const float sigma_spatial, const float sigma_range,
                                                 const int radius,
                                                 const size_t stride1, const size_t stride4,
                                                 const int xoff, const int yoff,
                                                 const int width, const int height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x + xoff;
  int y = blockIdx.y*blockDim.y + threadIdx.y + yoff;

  float4 p = prior[y*stride4+x];

  if(x<width && y<height)
  {
    float sum_g = 0.0f;
    float sum_val = 0.0f;

    for (int l=-radius; l<=radius; ++l)
    {
      for (int k=-radius; k<=radius; ++k)
      {
        int xx=x+k, yy=y+l;
        if(xx>=0 && yy>=0 && xx<width && yy<height)
        {
          float4 diff = p-prior[yy*stride4+xx];
          float g = expf(-((iu::sqr(x-xx)+iu::sqr(y-yy))/(2*iu::sqr(sigma_spatial)))
                         -(dot(diff,diff)/(2*iu::sqr(sigma_range))));
          sum_g += g;
          sum_val += g*src[y*stride1+x];
        }
      }
    }

    dst[y*stride1+x] = sum_val / IUMAX(1e-6f, sum_g);
  }
}

// ----------------------------------------------------------------------------
// kernel: bilateral filter kernel C4
__global__ void cuFilterBilateralKernel_32f_C4(const float4* src, float4* dst,
                                               const float4* prior,
                                               float sigma_spatial, const float sigma_range,
                                               const int radius, const size_t stride,
                                               const int xoff, const int yoff,
                                               const int width, const int height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x + xoff;
  int y = blockIdx.y*blockDim.y + threadIdx.y + yoff;

  int c = y*stride+x;
  float4 p = prior[c];

  if(x<width && y<height)
  {
    float sum_g = 0.0f;
    float4 sum_val = make_float4(0.0f);

    for (int l=-radius; l<=radius; ++l)
    {
      for (int k=-radius; k<=radius; ++k)
      {
        int xx=x+k, yy=y+l;
        if(xx>=0 && yy>=0 && xx<width && yy<height)
        {
          int cc = yy*stride+xx;
          float4 diff = p-prior[cc];
          float g = expf(-((iu::sqr(x-xx)+iu::sqr(y-yy))/(2*iu::sqr(sigma_spatial)))
                         -(dot(diff,diff)/(2*iu::sqr(sigma_range))));
          sum_g += g;
          sum_val += g*src[cc];
        }
      }
    }

    dst[c] = sum_val / IUMAX(1e-6f, sum_g);
  }

}


// ----------------------------------------------------------------------------
// wrapper: bilateral filter, C1
void cuFilterBilateral(const iu::ImageGpu_32f_C1* src, iu::ImageGpu_32f_C1* dst, const IuRect& roi,
                       const iu::ImageGpu_32f_C1* prior, const int iters,
                       const float sigma_spatial, const float sigma_range,
                       const int radius)
{
  float min,max;
  iu::minMax(src, src->roi(), min, max);
  printf("src min/max=%f/%f\n", min, max);
  iu::minMax(prior, src->roi(), min, max);
  printf("prior min/max=%f/%f\n", min, max);

  // fragmentation
  unsigned int block_size = 16;
  dim3 dimBlock(block_size, block_size);
  dim3 dimGrid(iu::divUp(roi.width, dimBlock.x), iu::divUp(roi.height, dimBlock.y));

  // filter iterations
  for (int iter=0; iter<iters; ++iter)
  {
    cuFilterBilateralKernel_32f_C1
        <<< dimGrid, dimBlock >>> (src->data(), dst->data(), prior->data(),
                                   sigma_spatial, sigma_range, radius,
                                   src->stride(), roi.x, roi.y, roi.width, roi.height);
  }

  IU_CUDA_CHECK();
}

// wrapper: bilateral filter, C1 and C4 prior
void cuFilterBilateral(const iu::ImageGpu_32f_C1* src, iu::ImageGpu_32f_C1* dst, const IuRect& roi,
                       const iu::ImageGpu_32f_C4* prior, const int iters,
                       const float sigma_spatial, const float sigma_range,
                       const int radius)
{
  // fragmentation
  unsigned int block_size = 16;
  dim3 dimBlock(block_size, block_size);
  dim3 dimGrid(iu::divUp(roi.width, dimBlock.x), iu::divUp(roi.height, dimBlock.y));

  // filter iterations
  for (int iter=0; iter<iters; ++iter)
  {
    cuFilterBilateralKernel_32f_C1C4
        <<< dimGrid, dimBlock >>> (src->data(), dst->data(), prior->data(),
                                   sigma_spatial, sigma_range, radius,
                                   src->stride(), prior->stride(),
                                   roi.x, roi.y, roi.width, roi.height);
  }

  IU_CUDA_CHECK();
}

// wrapper: bilateral filter, C4
void cuFilterBilateral(const iu::ImageGpu_32f_C4* src, iu::ImageGpu_32f_C4* dst, const IuRect& roi,
                       const iu::ImageGpu_32f_C4* prior, const int iters,
                       const float sigma_spatial, const float sigma_range,
                       const int radius)
{
  // fragmentation
  unsigned int block_size = 16;
  dim3 dimBlock(block_size, block_size);
  dim3 dimGrid(iu::divUp(roi.width, dimBlock.x), iu::divUp(roi.height, dimBlock.y));

  // filter iterations
  for (int iter=0; iter<iters; ++iter)
  {
    cuFilterBilateralKernel_32f_C4
        <<< dimGrid, dimBlock >>> (src->data(), dst->data(), prior->data(),
                                   sigma_spatial, sigma_range, radius,
                                   src->stride(), roi.x, roi.y, roi.width, roi.height);
  }

  IU_CUDA_CHECK();
}


} // namespace cu
} // namespace imp



#endif IMP_CU_BILATERAL_IMPL_CU
