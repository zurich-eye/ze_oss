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
#pragma once

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
/** filterMedian3x3 performs a median filter on a 3x3 window
 *
 */
template<typename Pixel>
void filterMedian3x3(ImageGpu<Pixel>& dst,
                     const ImageGpu<Pixel>& src);

//-----------------------------------------------------------------------------
/** filterGauss performs a gaussian smoothing filter on the given input image \a src
 * @param[out] dst Gauss filtered result image
 * @pram[in] src Input image on the GPU (CUDA memory)
 * @param[in] sigma Gaussian kernel standard deviation
 * @param[in] kernel_size Gaussian filter kernel size. (if default (0) computed automatically)
 * @param[inout] tmp_image optinal temp. image to avoid memory reallocation for multiple calls of the Gaussian filtering
 */
template<typename Pixel>
void filterGauss(ImageGpu<Pixel>& dst,
                 const ImageGpu<Pixel>& src,
                 float sigma, int kernel_size=0,
                 ImageGpuPtr<Pixel> tmp_img=nullptr);
//                 cudaStream_t stream);

template<typename Pixel>
void filterGauss(ImageGpu<Pixel>& dst,
                 const Texture2D& src_tex,
                 float sigma, int kernel_size=0,
                 ImageGpuPtr<Pixel> tmp_img=nullptr);
//                 cudaStream_t stream);

} // namespace cu
} // namespace ze
