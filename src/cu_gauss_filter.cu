#ifndef IMP_CU_GAUSS_IMPL_CU
#define IMP_CU_GAUSS_IMPL_CU

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

//-----------------------------------------------------------------------------
/** Perform a convolution with an gaussian smoothing kernel
 * @param dst          pointer to output image (linear memory)
 * @param stride       length of image row [pixels]
 * @param xoff         x-coordinate offset where to start the region [pixels]
 * @param yoff         y-coordinate offset where to start the region [pixels]
 * @param width        width of region [pixels]
 * @param height       height of region [pixels]
 * @param kernel_size  lenght of the smoothing kernel [pixels]
 * @param horizontal   defines the direction of convolution
 */
template<typename Pixel>
__global__ void k_gauss(Pixel* dst, const size_t stride,
                        const int xoff, const int yoff,
                        const int width, const int height,
                        Texture2D src_tex, int kernel_size, float c0,
                        float c1, bool horizontal=true)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;
  const size_t out_idx = y*stride+x;

  if(x>=0 && y>= 0 && x<width && y<height)
  {
    x += xoff;
    y += yoff;

    float sum = 0.0f;
    int half_kernel_elements = (kernel_size - 1) / 2;

    Pixel texel_c, texel;
    tex2DFetch(texel_c, src_tex, x, y);

    if (horizontal)
    {
      // convolve horizontally
      float g2 = c1 * c1;
      sum = c0 * texel_c;
      float sum_coeff = c0;

      for (int i = 1; i <= half_kernel_elements; i++)
      {
        c0 *= c1;
        c1 *= g2;
        int cur_x = max(0, min(width-1, x+i));
        tex2DFetch(texel, src_tex, cur_x, y);
        sum += c0 * texel;
        cur_x = max(0, min(width-1, x-i));
        tex2DFetch(texel, src_tex, cur_x, y);
        sum += c0 * texel;
        sum_coeff += 2.0f*c0;
      }
      dst[out_idx] = sum/sum_coeff;
    }
    else
    {
      // convolve vertically
      float g2 = c1 * c1;
      sum = c0 * texel_c;
      float sum_coeff = c0;

      for (int j = 1; j <= half_kernel_elements; j++)
      {
        c0 *= c1;
        c1 *= g2;
        float cur_y = max(0, min(height-1, y+j));
        tex2DFetch(texel, src_tex, x, cur_y);
        sum += c0 * texel;
        cur_y = max(0, min(height-1, y-j));
        tex2DFetch(texel, src_tex, x, cur_y);
        sum += c0 *  texel;
        sum_coeff += 2.0f*c0;
      }
      dst[out_idx] = sum/sum_coeff;
    }
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void filterGauss(ImageGpu<Pixel, pixel_type>& dst,
                 const Texture2D& src_tex,
                 float sigma, int kernel_size,
                 ImageGpuPtr<Pixel, pixel_type> tmp_img)
//                 cudaStream_t stream);
{
  Roi2u roi = dst.roi();

  if (kernel_size == 0)
    kernel_size = max(5, static_cast<int>(std::ceil(sigma*3)*2 + 1));
  if (kernel_size % 2 == 0)
    ++kernel_size;

  // temporary variable for filtering (separabel kernel!)
  if (!tmp_img || dst.roi().size() != tmp_img->size());
  {
    tmp_img.reset(new ImageGpu<Pixel, pixel_type>(roi.size()));
  }

  // fragmentation
  Fragmentation<> frag(roi);

  float c0 = 1.0f / (std::sqrt(2.0f * M_PI)*sigma);
  float c1 = std::exp(-0.5f / (sigma * sigma));

  // Convolve horizontally
  k_gauss
      <<<
        frag.dimGrid, frag.dimBlock//, 0, stream
      >>> (tmp_img->data(), tmp_img->stride(),
           roi.x(), roi.y(), tmp_img->width(), tmp_img->height(),
           src_tex, /*sigma, */kernel_size, c0, c1, false);

  IMP_CUDA_CHECK();
  std::shared_ptr<Texture2D> tmp_tex =
      tmp_img->genTexture(false,(tmp_img->bitDepth()<32) ? cudaFilterModePoint
                                                         : cudaFilterModeLinear);
  IMP_CUDA_CHECK();

  // Convolve vertically
  k_gauss
      <<<
        frag.dimGrid, frag.dimBlock//, 0, stream
      >>> (dst.data(), dst.stride(),
           roi.x(), roi.y(), roi.width(), roi.height(),
           *tmp_tex, /*sigma, */kernel_size, c0, c1, true);

  IMP_CUDA_CHECK();
}


//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void filterGauss(ImageGpu<Pixel, pixel_type>& dst,
                 const ImageGpu<Pixel, pixel_type>& src,
                 float sigma, int kernel_size,
                 ImageGpuPtr<Pixel, pixel_type> tmp_img)
//                 cudaStream_t stream);
{
  std::shared_ptr<Texture2D> src_tex =
      src.genTexture(false,(src.bitDepth()<32) ? cudaFilterModePoint
                                               : cudaFilterModeLinear);
  imp::Roi2u roi = src.roi();
  if (dst.roi().size() != roi.size())
     dst.setRoi(roi);

  imp::cu::filterGauss(dst, *src_tex, sigma, kernel_size, tmp_img);

  IMP_CUDA_CHECK();
}


//==============================================================================
//
// template instantiations for all our ima ge types
//

template void filterGauss(ImageGpu8uC1& dst, const ImageGpu8uC1& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu8uC1> tmp_imp);
template void filterGauss(ImageGpu8uC2& dst, const ImageGpu8uC2& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu8uC2> tmp_imp);
template void filterGauss(ImageGpu8uC4& dst, const ImageGpu8uC4& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu8uC4> tmp_imp);

template void filterGauss(ImageGpu16uC1& dst, const ImageGpu16uC1& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu16uC1> tmp_imp);
template void filterGauss(ImageGpu16uC2& dst, const ImageGpu16uC2& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu16uC2> tmp_imp);
template void filterGauss(ImageGpu16uC4& dst, const ImageGpu16uC4& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu16uC4> tmp_imp);

template void filterGauss(ImageGpu32sC1& dst, const ImageGpu32sC1& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32sC1> tmp_imp);
template void filterGauss(ImageGpu32sC2& dst, const ImageGpu32sC2& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32sC2> tmp_imp);
template void filterGauss(ImageGpu32sC4& dst, const ImageGpu32sC4& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32sC4> tmp_imp);

template void filterGauss(ImageGpu32fC1& dst, const ImageGpu32fC1& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32fC1> tmp_imp);
template void filterGauss(ImageGpu32fC2& dst, const ImageGpu32fC2& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32fC2> tmp_imp);
template void filterGauss(ImageGpu32fC4& dst, const ImageGpu32fC4& src, float sigma, int kernel_size, std::shared_ptr<ImageGpu32fC4> tmp_imp);


} // namespace cu
} // namespace imp



#endif // IMP_CU_GAUSS_IMPL_CU
