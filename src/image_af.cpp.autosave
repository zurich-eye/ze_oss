#include <imp/bridge/af/image_af.hpp>

namespace ze {

ImageAF32fC1::ImageAF32fC1(uint32_t width, uint32_t height)
  : ze::ImageRaw32fC1(width, height)
{
  af::array arr_color = af::loadImage("/home/mpi/workspace/arrayfire/assets/examples/images/man.jpg", true);
  arr_ = af::colorSpace(arr_color, AF_GRAY, AF_RGB);
}

} // ze namespace
