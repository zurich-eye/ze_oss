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
__global__
void k_sumCol(
    Pixel* d_col_sums,
    std::uint32_t roi_x,
    std::uint32_t roi_y,
    std::uint32_t roi_width,
    std::uint32_t roi_height,
    Texture2D img_tex)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  if (x<roi_width)
  {
    float xx = x + roi_x;

    Pixel col_sum{0};
    Pixel val;
    for (float yy = roi_y; yy<roi_y+roi_height; ++yy)
    {
      tex2DFetch(val, img_tex, xx, yy);
      col_sum += val;
    }
    d_col_sums[x] = col_sum;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel sum(const Texture2D& img_tex, const ze::Roi2u& roi)
{
  Fragmentation<512,1,1> frag(roi.width(), 1);
  Pixel sum_val;
  ze::cu::LinearMemory<Pixel> d_col_sums(roi.width());
  IMP_CUDA_CHECK();

  k_sumCol
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (d_col_sums.data(), roi.x(), roi.y(), roi.width(), roi.height(), img_tex);
  IMP_CUDA_CHECK();

  ze::LinearMemory<Pixel> h_col_sums(roi.width());
  d_col_sums.copyTo(h_col_sums);

  sum_val = h_col_sums(0);
  for (uint32_t i=1u; i<roi.width(); i++)
  {
    sum_val += h_col_sums(i);
  }

  IMP_CUDA_CHECK();
  return sum_val;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel sum(const ImageGpu<Pixel>& img)
{
  return ze::cu::sum<Pixel>(*img.genTexture(), img.roi());
}

// template instantiations for all our image types
// TODO (MPI) only 32fC1 is currently supported

/*
template ze::Pixel8uC1 sum(const ImageGpu8uC1& img);
template ze::Pixel8uC2 sum(const ImageGpu8uC2& img);
template ze::Pixel8uC4 sum(const ImageGpu8uC4& img);

template ze::Pixel16uC1 sum(const ImageGpu16uC1& im);
template ze::Pixel16uC2 sum(const ImageGpu16uC2& im);
template ze::Pixel16uC4 sum(const ImageGpu16uC4& im);

template ze::Pixel32sC1 sum(const ImageGpu32sC1& im);
template ze::Pixel32sC2 sum(const ImageGpu32sC2& im);
template ze::Pixel32sC4 sum(const ImageGpu32sC4& im);
*/

template ze::Pixel32fC1 sum(const ImageGpu32fC1& im);

/*
template ze::Pixel32fC2 sum(const ImageGpu32fC2& im);
template ze::Pixel32fC4 sum(const ImageGpu32fC4& im);
*/

} // namespace cu
} // namespace ze
