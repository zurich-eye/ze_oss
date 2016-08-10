#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/cameras/camera_models.h>

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
