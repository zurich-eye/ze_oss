#include <imp/bridge/af/feature_detection.hpp>

namespace ze {
namespace cu {

af::array createFromImp(const ImageGpu32fC1& in_img)
{
  af::array a = af::createStridedArray(
        in_img.cuData(), 0,
        af::dim4(
          in_img.width(),
          in_img.height(),
          1, 1),
        af::dim4(
          1, in_img.stride(),
          1, 1),
        f32,
        afDevice);
  return a;
}

} // cu namespace
} // ze namespace
