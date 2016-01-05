#include <imp/cu_imgproc/cu_resample.cuh>

#include <memory>
#include <cstdint>
#include <cmath>

#include <cuda_runtime.h>

#include <imp/core/types.hpp>
#include <imp/core/roi.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>

namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_resample(Pixel* d_dst, size_t stride,
                           std::uint32_t dst_width, std::uint32_t dst_height,
                           std::uint32_t roi_x, std::uint32_t roi_y,
                           float sf_x, float sf_y, Texture2D src_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x + roi_x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y + roi_y;
  if (x<dst_width && y<dst_height)
  {
    Pixel val;
    tex2DFetch(val, src_tex, x, y, sf_x, sf_y);
    d_dst[y*stride+x] = val;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void resample(ImageGpu<Pixel, pixel_type>& dst,
              const ImageGpu<Pixel, pixel_type>& src,
              imp::InterpolationMode interp, bool gauss_prefilter)
{
  imp::Roi2u src_roi = src.roi();
  imp::Roi2u dst_roi = dst.roi();

  // scale factor for x/y > 0 && < 1 (for multiplication with dst coords in the kernel!)
  float sf_x = static_cast<float>(src_roi.width()) / static_cast<float>(dst_roi.width());
  float sf_y = static_cast<float>(src_roi.height()) / static_cast<float>(dst_roi.height());

  cudaTextureFilterMode tex_filter_mode =
      (interp == InterpolationMode::linear) ? cudaFilterModeLinear
                                            : cudaFilterModePoint;
  if (src.bitDepth() < 32)
    tex_filter_mode = cudaFilterModePoint;

  std::shared_ptr<Texture2D> src_tex;
  std::unique_ptr<ImageGpu<Pixel,pixel_type>> filtered;
  if (gauss_prefilter)
  {
    float sf = .5f*(sf_x+sf_y);

    filtered.reset(new ImageGpu<Pixel, pixel_type>(src.size()));
    float sigma = 1/(3*sf) ;  // empirical magic
    std::uint16_t kernel_size = std::ceil(6.0f*sigma);
    if (kernel_size % 2 == 0)
      kernel_size++;

    imp::cu::filterGauss(*filtered, src, sigma, kernel_size);
    src_tex = filtered->genTexture(false, tex_filter_mode);
  }
  else
  {
    src_tex = src.genTexture(false, tex_filter_mode);
  }

  Fragmentation<> dst_frag(dst_roi.size());

  switch(interp)
  {
  case InterpolationMode::point:
  case InterpolationMode::linear:
    // fallthrough intended
    k_resample
        <<<
          dst_frag.dimGrid, dst_frag.dimBlock/*, 0, stream*/
        >>> (dst.data(), dst.stride(), dst.width(), dst.height(),
             dst_roi.x(), dst_roi.y(), sf_x , sf_y, *src_tex);
  break;
    //  case InterpolationMode::cubic:
    //    cuTransformCubicKernel_32f_C1
    //        <<< dimGridOut, dimBlock, 0, stream >>> (dst.data(), dst.stride(), dst.width(), dst.height(),
    //                                      sf_x , sf_y);
    //    break;
    //  case InterpolationMode::cubicSpline:
    //    cuTransformCubicSplineKernel_32f_C1
    //        <<< dimGridOut, dimBlock, 0, stream >>> (dst.data(), dst.stride(), dst.width(), dst.height(),
    //                                      sf_x , sf_y);
    //    break;
  default:
    IMP_THROW_EXCEPTION("unsupported interpolation type");
  }

  IMP_CUDA_CHECK();
}

//==============================================================================
//
// template instantiations for all our image types
//

template void resample(ImageGpu8uC1& dst, const ImageGpu8uC1& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu8uC2& dst, const ImageGpu8uC2& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu8uC4& dst, const ImageGpu8uC4& src, InterpolationMode interp, bool gauss_prefilter);

template void resample(ImageGpu16uC1& dst, const ImageGpu16uC1& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu16uC2& dst, const ImageGpu16uC2& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu16uC4& dst, const ImageGpu16uC4& src, InterpolationMode interp, bool gauss_prefilter);

template void resample(ImageGpu32sC1& dst, const ImageGpu32sC1& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu32sC2& dst, const ImageGpu32sC2& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu32sC4& dst, const ImageGpu32sC4& src, InterpolationMode interp, bool gauss_prefilter);

template void resample(ImageGpu32fC1& dst, const ImageGpu32fC1& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu32fC2& dst, const ImageGpu32fC2& src, InterpolationMode interp, bool gauss_prefilter);
template void resample(ImageGpu32fC4& dst, const ImageGpu32fC4& src, InterpolationMode interp, bool gauss_prefilter);


} // namespace cu
} // namespace imp
