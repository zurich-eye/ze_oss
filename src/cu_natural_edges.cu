#ifndef IM_CU_EDGE_DETECTORS_NATURAL_EDGES_IMPL_CUH
#define IM_CU_EDGE_DETECTORS_NATURAL_EDGES_IMPL_CUH

#include <imp/cu_imgproc/edge_detectors.cuh>

#include <cstdint>
#include <cuda_runtime.h>

#include <imp/core/types.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/roi.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_imgproc/cu_image_filter.cuh>

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
__device__ __forceinline__ void d_calcNaturalEdge(Pixel1<T>& g,
    const Pixel& dx, const Pixel& dy, const float alpha, const float q)
{
  float norm = sqrtf(sqr(dx) + sqr(dy));
  g = max(1e-3f, exp(-alpha*pow(norm,q)));
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
__device__ __forceinline__ void d_calcNaturalEdge(Pixel2<T>& g,
    const Pixel& dx, const Pixel& dy, const float alpha, const float q)
{
  g.x = max(1e-3f, exp(-alpha*pow(dx,q)));
  g.y = max(1e-3f, exp(-alpha*pow(dy,q)));
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
__device__ __forceinline__ void d_calcNaturalEdge(Pixel3<T>& g,
    const Pixel& dx, const Pixel& dy, const float alpha, const float q)
{
  float norm = sqrtf(sqr(dx) + sqr(dy));
  float n1 = 1.0f, n2 = 0.0f;
  float dt = 1e-6;
  if (norm > dt)
  {
    n1 = dx/norm;
    n2 = dy/norm;
  }
  // orthogonal to the normal
  float n1_ = n2;
  float n2_ = -n1;

  float w = max(1e-3f, exp(-alpha*pow(norm,q)));
  g.x = n1_*n1_ + w*n1*n1;
  g.y = n2_*n2_ + w*n2*n2;
  g.z = n1_*n2_ + w*n1*n2;
}

//------------------------------------------------------------------------------
template<typename Pixel, typename EdgePixel>
__global__ void k_naturalEdges(EdgePixel *g, const size_t stride,
                               const float alpha, const float q,
                               const std::uint32_t xoff, const std::uint32_t yoff,
                               const std::uint32_t width, const std::uint32_t height,
                               Texture2D src_tex)
{
  const int x = blockIdx.x*blockDim.x + threadIdx.x;
  const int y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<width && y<height)
  {
    Pixel ic, ixp, iyp;
    tex2DFetch(ic, src_tex, x, y);
    tex2DFetch(ixp, src_tex, x+1.f, y);
    tex2DFetch(iyp, src_tex, x, y+1.f);

    // calculate finite derivatives
    Pixel dx = ixp - ic;
    if (x >= width-1)
      dx = 0.0f;
    Pixel dy = iyp - ic;
    if (y >= height-1)
      dy = 0.0f;

    d_calcNaturalEdge(g[y*stride+x], dx, dy, alpha, q);
  }
}

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void naturalEdges(ImageGpu<Pixel, pixel_type>& dst,
                  const ImageGpu<Pixel, pixel_type>& src,
                  float sigma, float alpha, float q,
                  ImageGpuPtr<Pixel, pixel_type> tmp_denoised)
{
  Roi2u roi = src.roi();
  dst.setRoi(roi);

  // temporary variable for filtering (separabel kernel!)
  if (!tmp_denoised || src.size() != tmp_denoised->size())
  {
    tmp_denoised.reset(new ImageGpu<Pixel, pixel_type>(roi.size()));
  }
  else
  {
    tmp_denoised->setRoi(roi);
  }

  imp::cu::filterGauss(*tmp_denoised, src, sigma);

  std::shared_ptr<Texture2D> src_tex =
      tmp_denoised->genTexture(
        false, (tmp_denoised->bitDepth()<32) ? cudaFilterModePoint
                                             : cudaFilterModeLinear);
  IMP_CUDA_CHECK();

  Fragmentation<> frag(roi);

  k_naturalEdges<Pixel, Pixel>
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (dst.data(), dst.stride(),
           alpha, q,
           roi.x(), roi.y(), roi.width(), roi.height(), *src_tex);

  IMP_CUDA_CHECK();
}

//==============================================================================
//
// template instantiations for all our image types
//

template void naturalEdges(ImageGpu8uC1& dst, const ImageGpu8uC1& src, float sigma, float alpha, float q, ImageGpu8uC1::Ptr tmp_denoised);
//template void naturalEdges(ImageGpu8uC2* dst, ImageGpu8uC1* src, float sigma, float alpha, float q);
//template void naturalEdges(ImageGpu8uC3* dst, ImageGpu8uC1* src, float sigma, float alpha, float q);
//template void naturalEdges(ImageGpu8uC4* dst, ImageGpu8uC1* src, float sigma, float alpha, float q);

template void naturalEdges(ImageGpu32fC1& dst, const ImageGpu32fC1& src, float sigma, float alpha, float q, std::shared_ptr<ImageGpu32fC1> tmp_denoised);
//template void naturalEdges(ImageGpu32fC2* dst, ImageGpu32fC1* src, float sigma, float alpha, float q);
//template void naturalEdges(ImageGpu32fC3* dst, ImageGpu32fC1* src, float sigma, float alpha, float q);
//template void naturalEdges(ImageGpu32fC4* dst, ImageGpu32fC1* src, float sigma, float alpha, float q);


} // namespace cu
} // namespace imp

#endif // IM_CU_EDGE_DETECTORS_NATURAL_EDGES_IMPL_CUH
