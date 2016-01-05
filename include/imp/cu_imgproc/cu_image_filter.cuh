#ifndef IMP_CU_IMAGE_FILTER_CUH
#define IMP_CU_IMAGE_FILTER_CUH

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
/** filterMedian3x3 performs a median filter on a 3x3 window
 *
 */
template<typename Pixel, imp::PixelType pixel_type>
void filterMedian3x3(ImageGpu<Pixel, pixel_type>& dst,
                     const ImageGpu<Pixel, pixel_type>& src);

//-----------------------------------------------------------------------------
/** filterGauss performs a gaussian smoothing filter on the given input image \a src
 * @param[out] dst Gauss filtered result image
 * @pram[in] src Input image on the GPU (CUDA memory)
 * @param[in] sigma Gaussian kernel standard deviation
 * @param[in] kernel_size Gaussian filter kernel size. (if default (0) computed automatically)
 * @param[inout] tmp_image optinal temp. image to avoid memory reallocation for multiple calls of the Gaussian filtering
 */
template<typename Pixel, imp::PixelType pixel_type>
void filterGauss(ImageGpu<Pixel, pixel_type>& dst,
                 const ImageGpu<Pixel, pixel_type>& src,
                 float sigma, int kernel_size=0,
                 ImageGpuPtr<Pixel, pixel_type> tmp_img=nullptr);
//                 cudaStream_t stream);

template<typename Pixel, imp::PixelType pixel_type>
void filterGauss(ImageGpu<Pixel, pixel_type>& dst,
                 const Texture2D& src_tex,
                 float sigma, int kernel_size=0,
                 ImageGpuPtr<Pixel, pixel_type> tmp_img=nullptr);
//                 cudaStream_t stream);

} // namespace cu
} // namespace imp

#endif // IMP_CU_IMAGE_FILTER_CUH
