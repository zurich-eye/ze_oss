#ifndef IMP_CU_OCCLUSION_KERNEL
#define IMP_CU_OCCLUSION_KERNEL

#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_k_derivative.cuh>
#include <imp/cuda_toolkit/helper_math.h>


namespace imp {
namespace cu {


//------------------------------------------------------------------------------
__global__ void k_occlusionCandidatesUniqunessMapping(
    float* occ, size_t stride, std::uint32_t width, std::uint32_t height,
    Texture2D disp_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;

  const int wx = x + static_cast<int>(disp_tex.fetch<float>(x,y) + 0.5f);

  if (wx>0 && wx<width && y<height)
  {
    atomicAdd(&occ[y*stride+wx], 1.f);
  }
}

template<typename Pixel>
__host__ __device__ Pixel clamp(const Pixel& in, const Pixel& low, const Pixel& high)
{
  return max(low, min(high, in));
}

//------------------------------------------------------------------------------
__global__ void k_clampOcclusion(
    float* occ, size_t stride, std::uint32_t width, std::uint32_t height,
    Texture2D occ_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;
  if (x<width && y<height)
  {
    occ[y*stride+x] = 1.f - clamp(occ_tex.fetch<float>(x,y)-1.f, 0.f, 1.f);
  }
}


}
}

#endif
