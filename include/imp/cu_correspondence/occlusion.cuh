#ifndef IMP_CU_OCCLUSION_CUH
#define IMP_CU_OCCLUSION_CUH

#include <memory>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace imp {
namespace cu {

void occlusionCandidatesUniqunessMapping(ImageGpu32fC1::Ptr occ,
                                         const ImageGpu32fC1::Ptr& disp);

} // namespace cu
} // namespace imp


#endif // IMP_CU_OCCLUSION_CUH
