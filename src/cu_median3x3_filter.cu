#ifndef IMP_CU_MEDIAN3X3_IMPL_CU
#define IMP_CU_MEDIAN3X3_IMPL_CU

#include <imp/cu_imgproc/cu_image_filter.cuh>

#include <cstdint>
#include <cfloat>
#include <cuda_runtime.h>

#include <imp/core/types.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/roi.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>



namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void  k_median3x3(Pixel* dst, const size_t stride,
                             const std::uint32_t xoff, const std::uint32_t yoff,
                             const std::uint32_t width, const std::uint32_t height,
                             Texture2D src_tex)
{
  int x = blockIdx.x*blockDim.x + threadIdx.x;
  int y = blockIdx.y*blockDim.y + threadIdx.y;

  if(x>=0 && y>= 0 && x<width && y<height)
  {
    x += xoff;
    y += yoff;

    // shared mem coords
    const int tx = threadIdx.x+1;
    const int ty = threadIdx.y+1;
    // we have a 3x3 kernel, so our width of the shared memory (shp) is blockDim.x + 2!
    const int shp = blockDim.x + 2;
    const int shc = ty*shp + tx;
    extern __shared__ float sh_in[];

    // Load input 3x3 block into shared memory
    // Note: the FLT_MAX prevents us from overemphasizing the border pixels if they are outliers!
    {
      // for each thread: copy the data of the current input position to shared mem
      Pixel texel;
      tex2DFetch(texel, src_tex, x, y);
      sh_in[shc] = texel;

      /////////////////////////////////////////////////////////////////////////////
      // boundary conditions
      /////////////////////////////////////////////////////////////////////////////
      if (x == 0) // at left image border
      {
        if (y == 0)
          sh_in[shc-shp-1] = FLT_MAX; // left-upper corner (image)
        else if (ty == 1)
        {
          // left-upper corner (block)
          tex2DFetch(texel, src_tex, x, y-1.f);
          sh_in[shc-shp-1] = texel;
        }

        sh_in[shc-1] = sh_in[shc];     // left border (image)

        if (y == height-1)
          sh_in[shc+shp-1] = FLT_MAX; // left-lower corner (image)
        else if (ty == blockDim.y)
        {
          tex2DFetch(texel, src_tex, x, y+1);
          sh_in[shc+shp-1] = texel; // left-lower corner (block)
        }
      }
      else if (tx == 1) // at left block border (inside image w.r.t x)
      {
        if (y == 0)
        {
          tex2DFetch(texel, src_tex, x-1, y);
          sh_in[shc-shp-1] = texel; // left-upper corner (block, outside)
        }
        else if (ty == 1)
        {
          tex2DFetch(texel, src_tex, x-1, y-1);
          sh_in[shc-shp-1] = texel; // left-upper corner (block, inside)
        }

        tex2DFetch(texel, src_tex, x-1, y);
        sh_in[shc-1] = texel; // left border (block)

        if (y == height-1)
        {
          tex2DFetch(texel, src_tex, x-1, y);
          sh_in[shc+shp-1] = texel; // left-lower corner (block, outside)
        }
        else if (ty == blockDim.y)
        {
          tex2DFetch(texel, src_tex, x-1, y+1);
          sh_in[shc+shp-1] = texel; // left-lower corner (block, inside)
        }
      }


      if (x == width-1) // at right image border
      {
        if (y == 0)
          sh_in[shc-shp+1] = FLT_MAX; // right-upper corner (image)
        else if (ty == 1)
        {
          tex2DFetch(texel, src_tex, x, y-1);
          sh_in[shc-shp+1] = texel; // right-upper corner (block)
        }

        sh_in[shc+1] = sh_in[shc]; // right border (image)

        if (y == height-1)
          sh_in[shc+shp+1] = FLT_MAX; // right-lower corner (image)
        else if (ty == blockDim.y)
        {
          tex2DFetch(texel, src_tex, x, y+1);
          sh_in[shc+shp+1] = texel; // right-lower corner (block)
        }
      }
      else if (tx == blockDim.x) // at right block border (inside image w.r.t x)
      {
        if (y == 0)
        {
          tex2DFetch(texel, src_tex, x+1, y);
          sh_in[shc-shp+1] = texel; // right-upper corner (block, outside)
        }
        else if (ty == 1)
        {
          tex2DFetch(texel, src_tex, x+1, y-1);
          sh_in[shc-shp+1] = texel; // right-upper corner (block, inside)
        }

        tex2DFetch(texel, src_tex, x+1, y);
        sh_in[shc+1] = texel; // right border (block)

        if (y == height-1)
        {
          tex2DFetch(texel, src_tex, x+1, y);
          sh_in[shc+shp+1] = texel; // right-lower corner (block, outside)
        }
        else if (ty == blockDim.y)
        {
          tex2DFetch(texel, src_tex, x+1, y+1);
          sh_in[shc+shp+1] = texel; // right-lower corner (block, inside)
        }
      }

      if (y == 0)
        sh_in[shc-shp] = sh_in[shc]; // upper border (image)
      else if (ty == 1)
      {
        tex2DFetch(texel, src_tex, x, y-1);
        sh_in[shc-shp] = texel; // upper border (block)
      }

      if (y == height-1)
        sh_in[shc+shp] = sh_in[shc]; // lower border (image)
      else if (ty == blockDim.y)
      {
        tex2DFetch(texel, src_tex, x, y+1);
        sh_in[shc+shp] = texel; // lower border (block)
      }

      __syncthreads();
    }

    // in a sequence of nine elements, we have to remove four times the maximum from the sequence and need
    // a fifth calculated maximum which is the median!

    float maximum;
    {
      float vals[8];

      // first 'loop'
      vals[0] = fmin(sh_in[shc-shp-1], sh_in[shc-shp]);
      maximum = fmax(sh_in[shc-shp-1], sh_in[shc-shp]);
      vals[1] = fmin(maximum, sh_in[shc-shp+1]);
      maximum = fmax(maximum, sh_in[shc-shp+1]);
      vals[2] = fmin(maximum, sh_in[shc-1]);
      maximum = fmax(maximum, sh_in[shc-1]);
      vals[3] = fmin(maximum, sh_in[shc]);
      maximum = fmax(maximum, sh_in[shc]);
      vals[4] = fmin(maximum, sh_in[shc+1]);
      maximum = fmax(maximum, sh_in[shc+1]);
      vals[5] = fmin(maximum, sh_in[shc+shp-1]);
      maximum = fmax(maximum, sh_in[shc+shp-1]);
      vals[6] = fmin(maximum, sh_in[shc+shp]);
      maximum = fmax(maximum, sh_in[shc+shp]);
      vals[7] = fmin(maximum, sh_in[shc+shp+1]);
      maximum = fmax(maximum, sh_in[shc+shp+1]);

      // second 'loop'
      maximum = fmax(vals[0], vals[1]);
      vals[0] = fmin(vals[0], vals[1]);
      vals[1] = maximum;
      maximum = fmax(vals[1], vals[2]);
      vals[1] = fmin(vals[1], vals[2]);
      vals[2] = maximum;
      maximum = fmax(vals[2], vals[3]);
      vals[2] = fmin(vals[2], vals[3]);
      vals[3] = maximum;
      maximum = fmax(vals[3], vals[4]);
      vals[3] = fmin(vals[3], vals[4]);
      vals[4] = maximum;
      maximum = fmax(vals[4], vals[5]);
      vals[4] = fmin(vals[4], vals[5]);
      vals[5] = maximum;
      maximum = fmax(vals[5], vals[6]);
      vals[5] = fmin(vals[5], vals[6]);
      vals[6] = fmin(maximum, vals[7]);

      // third 'loop'
      maximum = fmax(vals[0], vals[1]);
      vals[0] = fmin(vals[0], vals[1]);
      vals[1] = maximum;
      maximum = fmax(vals[1], vals[2]);
      vals[1] = fmin(vals[1], vals[2]);
      vals[2] = maximum;
      maximum = fmax(vals[2], vals[3]);
      vals[2] = fmin(vals[2], vals[3]);
      vals[3] = maximum;
      maximum = fmax(vals[3], vals[4]);
      vals[3] = fmin(vals[3], vals[4]);
      vals[4] = maximum;
      maximum = fmax(vals[4], vals[5]);
      vals[4] = fmin(vals[4], vals[5]);
      vals[5] = fmin(maximum, vals[6]);

      // 4th 'loop'
      maximum = fmax(vals[0], vals[1]);
      vals[0] = fmin(vals[0], vals[1]);
      vals[1] = maximum;
      maximum = fmax(vals[1], vals[2]);
      vals[1] = fmin(vals[1], vals[2]);
      vals[2] = maximum;
      maximum = fmax(vals[2], vals[3]);
      vals[2] = fmin(vals[2], vals[3]);
      vals[3] = maximum;
      maximum = fmax(vals[3], vals[4]);
      vals[3] = fmin(vals[3], vals[4]);
      vals[4] = fmin(maximum, vals[5]);

      // 5th 'loop'
      maximum = fmax(vals[0], vals[1]);
      maximum = fmax(maximum, vals[2]);
      maximum = fmax(maximum, vals[3]);
      maximum = fmax(maximum, vals[4]);
    }

    dst[y*stride+x] = maximum;
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void filterMedian3x3(ImageGpu<Pixel, pixel_type>& dst,
                     const ImageGpu<Pixel, pixel_type>& src)
{
  std::shared_ptr<Texture2D> src_tex =
      src.genTexture(false, (src.bitDepth()<32) ? cudaFilterModePoint
                                                  : cudaFilterModeLinear);

  constexpr std::uint16_t block_size = 16;
  Fragmentation<block_size, block_size> frag(src.roi());
  size_t shared_size = (block_size+2)*(block_size+2)*sizeof(float);

  Roi2u roi = src.roi();
  dst.setRoi(roi);

  k_median3x3
      <<<
        frag.dimGrid, frag.dimBlock, shared_size
      >>> (
          dst.data(), dst.stride(),
          roi.x(), roi.y(), roi.width(), roi.height(), *src_tex);

  IMP_CUDA_CHECK();
}

//==============================================================================
//
// template instantiations for all our image types
//

template void filterMedian3x3(ImageGpu8uC1& dst, const ImageGpu8uC1& src);
template void filterMedian3x3(ImageGpu8uC2& dst, const ImageGpu8uC2& src);
template void filterMedian3x3(ImageGpu8uC4& dst, const ImageGpu8uC4& src);

template void filterMedian3x3(ImageGpu16uC1& dst, const ImageGpu16uC1& src);
template void filterMedian3x3(ImageGpu16uC2& dst, const ImageGpu16uC2& src);
template void filterMedian3x3(ImageGpu16uC4& dst, const ImageGpu16uC4& src);

template void filterMedian3x3(ImageGpu32sC1& dst, const ImageGpu32sC1& src);
template void filterMedian3x3(ImageGpu32sC2& dst, const ImageGpu32sC2& src);
template void filterMedian3x3(ImageGpu32sC4& dst, const ImageGpu32sC4& src);

template void filterMedian3x3(ImageGpu32fC1& dst, const ImageGpu32fC1& src);
template void filterMedian3x3(ImageGpu32fC2& dst, const ImageGpu32fC2& src);
template void filterMedian3x3(ImageGpu32fC4& dst, const ImageGpu32fC4& src);


} // namespace cu
} // namespace imp



#endif // IMP_CU_MEDIAN3X3_IMPL_CU
