#ifndef IMP_CU_RESAMPLE_CUH
#define IMP_CU_RESAMPLE_CUH

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

/**
 * @brief Image resampling from \a src to \a dst image
 */
template<typename Pixel>
void resample(ImageGpu<Pixel>& dst,
              const ImageGpu<Pixel>& src,
              InterpolationMode interp = InterpolationMode::Linear,
              bool gauss_prefilter=false);


} // namespace cu
} // namespace ze

#endif // IMP_CU_RESAMPLE_CUH
