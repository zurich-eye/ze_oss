#include <imp/bridge/af/image_af.hpp>

namespace ze {

ImageAF32fC1::ImageAF32fC1(uint32_t width, uint32_t height)
  : ImageBase(PixelType::i32fC1, sizeof(float), ze::PixelOrder::gray),
    arr_(width, height, f32)
{
  af::array arr_color = af::loadImage("/home/mpi/workspace/arrayfire/assets/examples/images/man.jpg", true);
  arr_ = af::colorSpace(arr_color, AF_GRAY, AF_RGB);
}

} // ze namespace
