#ifndef IMP_CU_MIN_MAX_IMPL_CU
#define IMP_CU_MIN_MAX_IMPL_CU

#include <imp/core/linearmemory.hpp>
#include <imp/core/image.hpp>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_exception.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>

namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_minMax(Pixel* d_col_mins, Pixel* d_col_maxs,
                         std::uint32_t roi_x, std::uint32_t roi_y,
                         std::uint32_t roi_width, std::uint32_t roi_height,
                         Texture2D img_tex)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;

  if (x<roi_width)
  {
    float xx = x + roi_x;
    float yy = roi_y;

    Pixel cur_min, cur_max;
    img_tex.fetch(cur_min, xx, ++yy);
    cur_max = cur_min;

    Pixel val;
    for (; yy<roi_y+roi_height; ++yy)
    {
      img_tex.fetch(val, xx, yy);
      if (val<cur_min) cur_min = val;
      if (val>cur_max) cur_max = val;
    }

    d_col_mins[x] = cur_min;
    d_col_maxs[x] = cur_max;
  }
}


//-----------------------------------------------------------------------------
template<typename Pixel>
void minMax(const Texture2D& img_tex, Pixel& min_val, Pixel& max_val, const imp::Roi2u& roi)
{
  Fragmentation<512,1,1> frag(roi.width(), 1);

  imp::cu::LinearMemory<Pixel> d_col_mins(roi.width());
  imp::cu::LinearMemory<Pixel> d_col_maxs(roi.width());
  IMP_CUDA_CHECK();

  k_minMax
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (d_col_mins.data(), d_col_maxs.data(),
           roi.x(), roi.y(), roi.width(), roi.height(), img_tex);
  IMP_CUDA_CHECK();

  imp::LinearMemory<Pixel> h_col_mins(roi.width());
  imp::LinearMemory<Pixel> h_col_maxs(roi.width());
  d_col_mins.copyTo(h_col_mins);
  d_col_maxs.copyTo(h_col_maxs);

  min_val = h_col_mins(0);
  max_val = h_col_maxs(0);

  for (auto i=1u; i<roi.width(); ++i)
  {
    min_val = imp::cu::min<Pixel>(min_val, h_col_mins(i));
    max_val = imp::cu::max<Pixel>(max_val, h_col_maxs(i));
  }

  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void minMax(const ImageGpu<Pixel, pixel_type>& img, Pixel& min_val, Pixel& max_val)
{
  IMP_CUDA_CHECK();

  std::unique_ptr<Texture2D> img_tex = img.genTexture();
  IMP_CUDA_CHECK();
  imp::Roi2u roi = img.roi();
  imp::cu::minMax(*img_tex, min_val, max_val, roi);
  IMP_CUDA_CHECK();
}


// template instantiations for all our image types
template void minMax(const ImageGpu8uC1& img, imp::Pixel8uC1& min, imp::Pixel8uC1& max);
template void minMax(const ImageGpu8uC2& img, imp::Pixel8uC2& min, imp::Pixel8uC2& max);
//template void minMax(const ImageGpu8uC3& img, imp::Pixel8uC3& min, imp::Pixel8uC3& max);
template void minMax(const ImageGpu8uC4& img, imp::Pixel8uC4& min, imp::Pixel8uC4& max);

template void minMax(const ImageGpu16uC1& img, imp::Pixel16uC1& min, imp::Pixel16uC1& max);
template void minMax(const ImageGpu16uC2& img, imp::Pixel16uC2& min, imp::Pixel16uC2& max);
//template void minMax(const ImageGpu16uC3& img, imp::Pixel16uC3& min, imp::Pixel16uC3& max);
template void minMax(const ImageGpu16uC4& img, imp::Pixel16uC4& min, imp::Pixel16uC4& max);

template void minMax(const ImageGpu32sC1& img, imp::Pixel32sC1& min, imp::Pixel32sC1& max);
template void minMax(const ImageGpu32sC2& img, imp::Pixel32sC2& min, imp::Pixel32sC2& max);
//template void minMax(const ImageGpu32sC3& img, imp::Pixel32sC3& min, imp::Pixel32sC3& max);
template void minMax(const ImageGpu32sC4& img, imp::Pixel32sC4& min, imp::Pixel32sC4& max);

template void minMax(const ImageGpu32fC1& img, imp::Pixel32fC1& min, imp::Pixel32fC1& max);
template void minMax(const ImageGpu32fC2& img, imp::Pixel32fC2& min, imp::Pixel32fC2& max);
//template void minMax(const ImageGpu32fC3& img, imp::Pixel32fC3& min, imp::Pixel32fC3& max);
template void minMax(const ImageGpu32fC4& img, imp::Pixel32fC4& min, imp::Pixel32fC4& max);

} // namespace cu
} // namespace imp

#endif // IMP_CU_MIN_MAX_IMPL_CU
