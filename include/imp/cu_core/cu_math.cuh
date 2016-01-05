#ifndef IMP_CU_MATH_CUH
#define IMP_CU_MATH_CUH

#include <memory>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace imp {
namespace cu {

/**
 * @brief Finding min and max pixel value of given image
 * @note For multi-channel images, the seperate channels are not handeled individually.
 */
template<typename Pixel, imp::PixelType pixel_type>
void minMax(const ImageGpu<Pixel, pixel_type>& img, Pixel& min, Pixel& max);

template<typename Pixel>
void minMax(const Texture2D& img_tex, Pixel& min, Pixel& max, const imp::Roi2u& roi);

//template<typename Pixel, imp::PixelType pixel_type>
//void sum(const ImageGpu<Pixel, pixel_type>& img, double& sum);

//template<typename Pixel>
//void sum(const Texture2D& img_tex, double& sum, const imp::Roi2u& roi);

} // namespace cu
} // namespace imp

#endif // IMP_CU_MATH_CUH

