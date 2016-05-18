#pragma once

#include <arrayfire.h>
#include <af/internal.h>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace ze {
namespace cu {

af::array createFromImp(const ImageGpu32fC1& in_img);

} // cu namespace
} // ze namespace
