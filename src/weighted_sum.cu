#include <type_traits>
#include <imp/core/image.hpp>
#include <imp/cu_core/cu_math.cuh>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
__global__ void k_weightedSum(Pixel* dst, uint32_t dst_stride,
                              uint32_t dst_roi_x, uint32_t dst_roi_y,
                              Texture2D src1, const float weight1,
                              Texture2D src2, const float weight2,
                              uint32_t src1_roi_x, uint32_t src1_roi_y,
                              uint32_t src2_roi_x, uint32_t src2_roi_y,
                              uint32_t roi_width, uint32_t roi_height)
{
  uint32_t x = blockIdx.x*blockDim.x + threadIdx.x;
  uint32_t y = blockIdx.y*blockDim.y + threadIdx.y;

  if (x<roi_width && y<roi_height)
  {
    Pixel src1_val, src2_val;
    tex2DFetch(src1_val, src1, x, y, 1.f, 1.f, src1_roi_x, src1_roi_y);
    tex2DFetch(src2_val, src2, x, y, 1.f, 1.f, src2_roi_x, src2_roi_y);
    if (std::is_integral<typename Pixel::T>::value)
    {
      dst[(y+dst_roi_y)*dst_stride + (x+dst_roi_x)] =
          static_cast<Pixel>(weight1 * src1_val + weight2 * src2_val + 0.5f);
    }
    else
    {
      dst[(y+dst_roi_y)*dst_stride + (x+dst_roi_x)] =
          weight1 * src1_val + weight2 * src2_val;
    }
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void weightedSum(ImageGpu<Pixel>& dst,
                 const Texture2D& src1_tex, const Roi2u src1_roi, const float& weight1,
                 const Texture2D& src2_tex, const Roi2u src2_roi, const float& weight2)
{
  CHECK_EQ(src1_roi.size(), src2_roi.size());
  CHECK_EQ(src1_roi.size(), dst.roi().size());

  Roi2u dst_roi = dst.roi();
  Fragmentation<> frag(dst_roi);

  k_weightedSum
      <<<
        frag.dimGrid, frag.dimBlock
      >>> (dst.data(), dst.stride(),
           dst.roi().x(), dst.roi().y(),
           src1_tex, weight1, src2_tex, weight2,
           src1_roi.x(), src1_roi.y(), src2_roi.x(), src2_roi.y(),
           dst_roi.width(), dst_roi.height());
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void weightedSum(ImageGpu<Pixel>& dst,
                 const ImageGpu<Pixel>& src1, const float& weight1,
                 const ImageGpu<Pixel>& src2, const float& weight2)
{
  std::shared_ptr<Texture2D> src1_tex = src1.genTexture(false, cudaFilterModePoint);
  std::shared_ptr<Texture2D> src2_tex = src2.genTexture(false, cudaFilterModePoint);
  weightedSum(dst,
              *src1_tex, src1.roi(), weight1,
              *src2_tex, src2.roi(), weight2);
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageGpuPtr<Pixel> weightedSum(const ImageGpu<Pixel>& src1, const float& weight1,
                               const ImageGpu<Pixel>& src2, const float& weight2)
{
  ImageGpuPtr<Pixel> dst = std::make_shared<ImageGpu<Pixel>>(src1.size());
  dst->setRoi(src1.roi());
  std::shared_ptr<Texture2D> src1_tex = src1.genTexture(false, cudaFilterModePoint);
  std::shared_ptr<Texture2D> src2_tex = src2.genTexture(false, cudaFilterModePoint);
  ze::cu::weightedSum(*dst,
                      *src1_tex, src1.roi(), weight1,
                      *src2_tex, src2.roi(), weight2);
  IMP_CUDA_CHECK();
  return dst;
}

// template instantiations for all our image types
template void weightedSum(ImageGpu8uC1& dst,
                          const Texture2D& src1_tex, const Roi2u src1_roi, const float& weight1,
                          const Texture2D& src2_tex, const Roi2u src2_roi, const float& weight2);
template void weightedSum(ImageGpu32fC1& dst,
                          const Texture2D& src1_tex, const Roi2u src1_roi, const float& weight1,
                          const Texture2D& src2_tex, const Roi2u src2_roi, const float& weight2);

template void weightedSum(ImageGpu8uC1& dst,
                          const ImageGpu8uC1& src1, const float& weight1,
                          const ImageGpu8uC1& src2, const float& weight2);
template void weightedSum(ImageGpu32fC1& dst,
                          const ImageGpu32fC1& src1, const float& weight1,
                          const ImageGpu32fC1& src2, const float& weight2);

template ImageGpu8uC1::Ptr weightedSum(
    const ImageGpu8uC1& src1, const float& weight1,
    const ImageGpu8uC1& src2, const float& weight2);
template ImageGpu32fC1::Ptr weightedSum(
    const ImageGpu32fC1& src1, const float& weight1,
    const ImageGpu32fC1& src2, const float& weight2);

} // namespace cu
} // namespace ze
