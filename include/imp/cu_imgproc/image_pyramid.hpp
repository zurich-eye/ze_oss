#pragma once

#include <imp/imgproc/image_pyramid.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_reduce.cuh>

namespace ze {

/*
// Factory method
template<typename Pixel, ze::PixelType pixel_type>
ImagePyramid<Pixel,pixel_type>::Ptr createPyramid(
    ImagePtr img_level0, InterpolationMode interp)
{
  // TODO sanity checks

  levels_.push_back(img_level0);
  Size2u sz0 =  img_level0->size();

  for (size_t i=1; i<num_levels_; ++i)
  {
    Size2u sz(static_cast<std::uint32_t>(sz0.width()*scale_factors_[i] + 0.5f),
              static_cast<std::uint32_t>(sz0.height()*scale_factors_[i] + 0.5f));

    // init level memory with either ImageGpu or ImageRaw
    if(img_level0->isGpuMemory())
    {
      using ImageGpu = typename ze::cu::ImageGpu<Pixel,pixel_type>;
      typename ImageGpu::Ptr img = std::make_shared<ImageGpu>(sz);
      typename ImageGpu::Ptr prev = std::dynamic_pointer_cast<ImageGpu>(levels_.back());
      ze::cu::reduce(*img, *prev, interp, true);
      levels_.push_back(img);
    }
    else
    {
      using ImageRaw = ze::ImageRaw<Pixel,pixel_type>;
      typename ImageRaw::Ptr img = std::make_shared<ImageRaw>(sz);
      //! @todo (MWE) cpu reduction
      throw ze::Exception("CPU reduction not yet implemented.", __FILE__, __FUNCTION__, __LINE__);
      levels_.push_back(img);
    }
  }
}
*/


} // namespace ze
