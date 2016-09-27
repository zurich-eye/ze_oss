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

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.hpp>

namespace ze {
namespace cu {

//! \brief A StereoRectifier can rectify images using the GPU
template<typename CameraModel,
         typename DistortionModel,
         typename Pixel>
class StereoRectifier
{
public:
  //! \brief StereoRectifier
  //! \param img_size The size of the images to rectify.
  //! \param camera_params The camera parameters [fx, fy, cx, cy]^T.
  //! \param transformed_camera_params The camera parameters
  //! [fx, fy, cx, cy]^T in the rectified reference frame.
  //! \param dist_coeffs Camera distortion coefficients.
  //! \param inv_H Inverse of the rectifying homography.
  StereoRectifier(const Size2u& img_size,
                  const Vector4& camera_params,
                  const Vector4& transformed_camera_params,
                  const Vector4& dist_coeffs,
                  const Matrix3& inv_H);

  ~StereoRectifier() = default;

  //! \brief Rectify
  //! \param dst Destination image to store rectification result in GPU memory.
  //! \param src The image to rectify in GPU memory
  void rectify(ImageGpu<Pixel>& dst,
               const ImageGpu<Pixel>& src) const;

  //! \brief Retrieves the computed undistortion-rectification map
  const ImageGpu32fC2& getUndistortRectifyMap() const;

private:
  ImageGpu32fC2 undistort_rectify_map_;   //!< The (2-channels) undistortion-rectification map
  Fragmentation<16, 16> fragm_;           //!< @todo (MPI) opptimize CUDA thread grid config
};

using EquidistStereoRectifier32fC1 = StereoRectifier<PinholeGeometry, EquidistantDistortion, Pixel32fC1>;
using RadTanStereoRectifier32fC1 = StereoRectifier<PinholeGeometry, RadialTangentialDistortion, Pixel32fC1>;

} // cu namespace
} // ze namespace
