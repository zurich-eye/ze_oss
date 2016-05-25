#ifndef IMP_CU_MATH_CUH
#define IMP_CU_MATH_CUH

#include <memory>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

/**
 * @brief Finding min and max pixel value of given image
 * @note For multi-channel images, the seperate channels are not handeled individually.
 */
template<typename Pixel>
void minMax(const ImageGpu<Pixel>& img, Pixel& min, Pixel& max);

template<typename Pixel>
void minMax(const Texture2D& img_tex, Pixel& min, Pixel& max, const ze::Roi2u& roi);

/**
 * @brief Computing the sum of all pixels
 * @note For multi-channel images, the seperate channels are not handeled individually.
 */
template<typename Pixel>
Pixel sum(const ImageGpu<Pixel>& img);

template<typename Pixel>
Pixel sum(const Texture2D& img_tex, const ze::Roi2u& roi);

/** Weighted sum of two images (dst not allocated).
 * \param src1 Source image 1.
 * \param weight1 Multiplicative weight of image 1.
 * \param src2 Source image 2.
 * \param weight1 Multiplicative weight of image 1.
 * \param dst Result image dst=weight1*src1 + weight1*src2.
 *
 * \note supported gpu: 8uC1, 32f_C1
 */
template<typename Pixel>
void weightedSum(ImageGpu<Pixel>& dst,
                 const ImageGpu<Pixel>& src1, const float& weight1,
                 const ImageGpu<Pixel>& src2, const float& weight2);

/** Weighted sum of two images (dst internally allocated).
 * \param src1 Source image 1.
 * \param weight1 Multiplicative weight of image 1.
 * \param src2 Source image 2.
 * \param weight1 Multiplicative weight of image 1.
 * \param dst Result image dst=weight1*src1 + weight1*src2.
 *
 * \note supported gpu: 8uC1, 32f_C1
 */
template<typename Pixel>
ImageGpuPtr<Pixel> weightedSum(const ImageGpu<Pixel>& src1, const float& weight1,
                               const ImageGpu<Pixel>& src2, const float& weight2);

} // namespace cu
} // namespace ze

#endif // IMP_CU_MATH_CUH

