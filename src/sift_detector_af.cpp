#include <imp/bridge/af/sift_detector_af.hpp>

namespace ze {

SiftDetectorAF::SiftDetectorAF(const SiftDetectorOptions& options, const Size2u& image_size)
  : AbstractDetector(image_size, DetectorType::Sift),
    options_(options)
{ }

uint32_t SiftDetectorAF::detect(const ImagePyramid8uC1 &pyr, KeypointsWrapper &keypoints)
{
  // Not implemented
  return 0;
}

uint32_t SiftDetectorAF::detect(const ImageAF8uC1& pyr, KeypointsWrapper& keypoints)
{
  af::features feat;
  af::array desc;
  af::sift(feat, desc, pyr.afArray(), 3, 0.04f, 10.0f, 1.6f, true, 1.f/256.f, 0.05f);
  return 0;
}

} // ze namespace
