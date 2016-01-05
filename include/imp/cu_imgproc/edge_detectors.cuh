#ifndef IM_CU_EDGE_DETECTORS_CUH
#define IM_CU_EDGE_DETECTORS_CUH

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>


namespace imp {
namespace cu {

/** Compute 'natural' image edges as a function like g = exp(-alpha*(norm)^q)
 * Dependending on the dimension of the edge image the function computes the edge's
 * magnitude (EdgePixel == Pixel1), its direction (EdgePixel == Pixel2) or a
 * 3-dimensional tensor (EdgePixel == Pixel3 or Pixel4).
 *
 * @todo (MWE) add a tmp image as input param so we don't have to allocate memory
 *             for the internal denoising all the time.
 */
template<typename Pixel, imp::PixelType pixel_type>
void naturalEdges(ImageGpu<Pixel, pixel_type>& dst,
                  const ImageGpu<Pixel, pixel_type>& src,
                  float sigma=1.0f, float alpha=10.f, float q=.5f,
                  ImageGpuPtr<Pixel, pixel_type> tmp_denoised=nullptr);

} // namespace cu
} // namespace imp

#endif // IM_CU_EDGE_DETECTORS_CUH
