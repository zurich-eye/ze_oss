#include <imp/core/linearmemory.hpp>
#include <imp/core/image.hpp>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_exception.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_minMax(Pixel* d_col_mins, Pixel* d_col_maxs,
                         uint32_t roi_x, uint32_t roi_y,
                         uint32_t roi_width, uint32_t roi_height,
                         Texture2D img_tex)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;

  if (x<roi_width)
  {
    float xx = x + roi_x;
    float yy = roi_y;

    Pixel cur_min, cur_max;
    tex2DFetch(cur_min, img_tex, xx, yy++);
    cur_max = cur_min;

    Pixel val;
    for (; yy<roi_y+roi_height; ++yy)
    {
      tex2DFetch(val, img_tex, xx, yy);
      if (val<cur_min) cur_min = val;
      if (val>cur_max) cur_max = val;
    }

    d_col_mins[x] = cur_min;
    d_col_maxs[x] = cur_max;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, typename SrcPixel>
__global__ void k_minMax(Pixel* d_col_mins, Pixel* d_col_maxs,
                         SrcPixel* src, size_t src_stride,
                         uint32_t roi_x, uint32_t roi_y,
                         uint32_t roi_width, uint32_t roi_height)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;

  if (x<roi_width)
  {
    int xx = x+roi_x;
    int yy = roi_y;


    Pixel cur_min, cur_max;
    Pixel val = (Pixel)src[yy++*src_stride+xx];
    cur_min = val;
    cur_max = val;
    for (; yy<roi_y+roi_height; ++yy)
    {
      val = (Pixel)src[yy*src_stride+xx];
      cur_min = ze::cu::min(cur_min, val);
      cur_max = ze::cu::max(cur_max, val);
    }

    d_col_mins[x] = cur_min;
    d_col_maxs[x] = cur_max;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void minMax(const Texture2D& img_tex, Pixel& min_val, Pixel& max_val, const ze::Roi2u& roi)
{
  Fragmentation<512,1> frag(roi.width(), 1);

  ze::cu::LinearMemory<Pixel> d_col_mins(roi.width());
  ze::cu::LinearMemory<Pixel> d_col_maxs(roi.width());
  IMP_CUDA_CHECK();
  d_col_mins.setValue(Pixel(0));
  d_col_maxs.setValue(Pixel(0));

  k_minMax
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (d_col_mins.data(), d_col_maxs.data(),
           roi.x(), roi.y(), roi.width(), roi.height(), img_tex);
  IMP_CUDA_CHECK();

  ze::LinearMemory<Pixel> h_col_mins(d_col_mins.length());
  ze::LinearMemory<Pixel> h_col_maxs(d_col_maxs.length());
  h_col_mins.setValue(Pixel(0));
  h_col_maxs.setValue(Pixel(0));

  d_col_mins.copyTo(h_col_mins);
  d_col_maxs.copyTo(h_col_maxs);

  min_val = h_col_mins(0);
  max_val = h_col_maxs(0);

  for (auto i=1u; i<roi.width(); ++i)
  {
    min_val = ze::cu::min(min_val, h_col_mins(i));
    max_val = ze::cu::max(max_val, h_col_maxs(i));
  }

  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void minMax(const ImageGpu<Pixel>& img, Pixel& min_val, Pixel& max_val)
{
  ze::cu::minMax(*img.genTexture(), min_val, max_val, img.roi());
  IMP_CUDA_CHECK();
}


// template instantiations for all our image types
template void minMax(const ImageGpu8uC1& img, ze::Pixel8uC1& min, ze::Pixel8uC1& max);
template void minMax(const ImageGpu8uC2& img, ze::Pixel8uC2& min, ze::Pixel8uC2& max);
//template void minMax(const ImageGpu8uC3& img, ze::Pixel8uC3& min, ze::Pixel8uC3& max);
template void minMax(const ImageGpu8uC4& img, ze::Pixel8uC4& min, ze::Pixel8uC4& max);

template void minMax(const ImageGpu16uC1& img, ze::Pixel16uC1& min, ze::Pixel16uC1& max);
template void minMax(const ImageGpu16uC2& img, ze::Pixel16uC2& min, ze::Pixel16uC2& max);
//template void minMax(const ImageGpu16uC3& img, ze::Pixel16uC3& min, ze::Pixel16uC3& max);
template void minMax(const ImageGpu16uC4& img, ze::Pixel16uC4& min, ze::Pixel16uC4& max);

template void minMax(const ImageGpu32sC1& img, ze::Pixel32sC1& min, ze::Pixel32sC1& max);
template void minMax(const ImageGpu32sC2& img, ze::Pixel32sC2& min, ze::Pixel32sC2& max);
//template void minMax(const ImageGpu32sC3& img, ze::Pixel32sC3& min, ze::Pixel32sC3& max);
template void minMax(const ImageGpu32sC4& img, ze::Pixel32sC4& min, ze::Pixel32sC4& max);

template void minMax(const ImageGpu32fC1& img, ze::Pixel32fC1& min, ze::Pixel32fC1& max);
template void minMax(const ImageGpu32fC2& img, ze::Pixel32fC2& min, ze::Pixel32fC2& max);
//template void minMax(const ImageGpu32fC3& img, ze::Pixel32fC3& min, ze::Pixel32fC3& max);
template void minMax(const ImageGpu32fC4& img, ze::Pixel32fC4& min, ze::Pixel32fC4& max);

} // namespace cu
} // namespace ze
