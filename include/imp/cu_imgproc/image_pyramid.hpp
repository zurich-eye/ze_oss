#pragma once

#include <imp/imgproc/image_pyramid.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_reduce.cuh>

namespace ze {

//! Image Pyramid Factory:
template<typename Pixel>
std::shared_ptr<ImagePyramid<Pixel>>
createImagePyramidGpu(
    typename Image<Pixel>::Ptr img_level0, FloatType scale_factor=0.5,
    uint32_t max_num_levels=UINT32_MAX, uint32_t size_bound=8u)
{
  // Sanity checks.
  CHECK(img_level0->isGpuMemory());
  using Pyr = ImagePyramid<Pixel>;
  auto pyr = std::make_shared<Pyr>(
        img_level0->size(), scale_factor, size_bound, max_num_levels);

  pyr->push_back(img_level0);
  Size2u sz0 =  img_level0->size();

  for (size_t i=1; i<pyr->numLevels(); ++i)
  {
    Size2u sz(static_cast<std::uint32_t>(sz0.width() * pyr->scaleFactor(i) + 0.5f),
              static_cast<std::uint32_t>(sz0.height() * pyr->scaleFactor(i) + 0.5f));

    // init level memory with either ImageGpu or ImageRaw
    using ImageGpu = typename ze::cu::ImageGpu<Pixel>;
    pyr->emplace_back(std::make_shared<ImageGpu>(sz));
    typename ImageGpu::Ptr prev = std::dynamic_pointer_cast<ImageGpu>(pyr->atShared(i-1));
    typename ImageGpu::Ptr img = std::dynamic_pointer_cast<ImageGpu>(pyr->atShared(i));
    ze::cu::reduce(*img, *prev, InterpolationMode::linear, true);
  }
  return pyr;
}

} // namespace ze
