#pragma once

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

/**
 * @brief Image reduction from \a src to \a dst image
 */
template<typename Pixel>
void reduce(ImageGpu<Pixel>& dst,
            const ImageGpu<Pixel>& src,
            InterpolationMode interp = InterpolationMode::Linear,
            bool gauss_prefilter=true);


// /**
//  * @brief Image reduction from \a src to \a dst image
//  */
// template<typename Pixel, imp::PixelType pixel_type>
// void resample(ImageGpu<Pixel, pixel_type>& dst,
//               const ImageGpu<Pixel, pixel_type>& src,
//               InterpolationMode interp = InterpolationMode::Linear,
//               bool gauss_prefilter=false);


} // namespace cu
} // namespace ze
