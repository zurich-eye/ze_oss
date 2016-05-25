#include <imp/bridge/af/fast_detector_af.hpp>

namespace ze {

FastDetectorAF::FastDetectorAF(const FastDetectorOptions &options, const Size2u& image_size)
  : FastDetector(options, image_size)
{ }

uint32_t FastDetectorAF::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& features)
{
  int capacity = features.px.cols() - features.num_detected;
  if (capacity <= 0)
  {
    VLOG(100) << "Have no capacity for more corners. Skip FAST detection.";
    return 0u;
  }
  for (size_t l=0; l<pyr.numLevels(); ++l)
  {
    const ImageAF8uC1& l_img = dynamic_cast<const ImageAF8uC1&>(pyr.at(l));
    af::features feat = af::fast(l_img.afArray(), 20.0f, 9, true, 0.05);
    printf("Level %lu, number of features %lu\n", l, feat.getNumFeatures());
  }

  //uint32_t num_features = feat.
  return 0;
}

} // ze namespace
