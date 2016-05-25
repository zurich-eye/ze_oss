#include <imp/bridge/af/fast_detector_af.hpp>

namespace ze {

FastDetectorAF::FastDetectorAF(const FastDetectorOptions &options, const Size2u& image_size)
  : FastDetector(options, image_size)
{ }

uint32_t FastDetectorAF::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& features)
{
  //! TODO (MPI) grid is not currently supported
  int capacity = features.px.cols() - features.num_detected;
  if (capacity <= 0)
  {
    VLOG(100) << "Have no capacity for more corners. Skip FAST detection.";
    return 0u;
  }
  for (uint8_t level=options_.min_level; level<=options_.max_level; ++level)
  {
    const FloatType scale = 1.0f / pyr.scaleFactor(level);
    const ImageAF8uC1& l_img =
        dynamic_cast<const ImageAF8uC1&>(pyr.at(level));
    af::features feat =
        af::fast(l_img.afArray(), options_.threshold, 10, true, 0.05);
    float* h_x = feat.getX().host<float>();
    float* h_y = feat.getY().host<float>();
    float* h_score = feat.getScore().host<float>();
    const size_t num_detected_l = feat.getNumFeatures();
    printf("Level %i, number of features %lu\n", level, num_detected_l);
    for (size_t f=0; f<num_detected_l; ++f)
    {
      const FloatType x = h_x[f] * scale;
      const FloatType y = h_y[f] * scale;
      const float score = h_score[f];
      if (!features.addKeypoint(
            x, y, score, level, 0.0f,
            static_cast<uint8_t>(DetectorType::Fast)))
      {
        break;
      }
    }
  }
  return features.num_detected;
}

} // ze namespace
