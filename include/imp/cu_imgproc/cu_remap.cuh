#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_texture.cuh>

namespace ze {
namespace cu {

template<typename T>
__global__
void k_remap(
    Pixel1<T>* dst,
    size_t dst_stride,
    const Pixel32fC2* map,
    size_t map_stride,
    std::uint32_t width,
    std::uint32_t height,
    Texture2D src)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x < width && y < height)
  {
    Pixel1<T> val;
    const Pixel32fC2& px = map[y*map_stride+x];
    tex2DFetch(val, src, px[0], px[1]);
    dst[y*dst_stride+x] = val;
  }
}

} // cu namespace
} // ze namespace
