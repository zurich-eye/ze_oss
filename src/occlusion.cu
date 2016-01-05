#include <imp/cu_correspondence/occlusion.cuh>

#include <cuda_runtime_api.h>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
__global__ void k_occlusionCandidatesUniqunessMapping(
    float* occ, size_t stride, std::uint32_t width, std::uint32_t height,
    Texture2D disp_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x /*+ roi_x*/;
  const int y = blockIdx.y*blockDim.y + threadIdx.y /*+ roi_y*/;

  const int wx = x + static_cast<int>(tex2DFetch<float>(disp_tex, x, y) + 0.5f);

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
    occ[y*stride+x] = 1.f - clamp(tex2DFetch<float>(occ_tex, x, y)-1.f, 0.f, 1.f);
  }
}

//------------------------------------------------------------------------------
void occlusionCandidatesUniqunessMapping(
    ImageGpu32fC1::Ptr occ,
    const ImageGpu32fC1::Ptr& disp)
{
  occ->setValue(0.0f);
  std::shared_ptr<imp::cu::Texture2D> disp_tex = disp->genTexture();
  imp::cu::Fragmentation<> frag(disp->size());
  k_occlusionCandidatesUniqunessMapping
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (occ->cuData(), occ->stride(), occ->width(), occ->height(),
           *disp_tex);

  // clamp the occlusions to get a nice mask with 0 and 1 entries
  std::shared_ptr<Texture2D> occ_tex = occ->genTexture();
  k_clampOcclusion
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (occ->cuData(), occ->stride(), occ->width(), occ->height(), *occ_tex);
}

} // namespace cu
} // namespace imp
