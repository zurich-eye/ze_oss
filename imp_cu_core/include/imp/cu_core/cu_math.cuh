// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

