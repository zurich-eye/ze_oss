#include <imp/bridge/af/orb_detector_af.hpp>

namespace ze {

OrbDetectorAF::OrbDetectorAF(const OrbDetectorOptions& options, const Size2u& image_size)
  : AbstractDetector(image_size, DetectorType::Orb),
    options_(options)
{ }

uint32_t OrbDetectorAF::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints)
{
  LOG(FATAL) << "Not implemented";
  return 0;
}

uint32_t OrbDetectorAF::detect(const ImageAF32fC1& im, OrbKeypointWrapper::Ptr& keypoints)
{
  af::features feat;
  af::array desc;
  af::orb(
        feat,
        desc,
        im.afArray(),
        options_.fast_threshold,
        options_.max_num_features,
        options_.scale_factor,
        options_.pyramid_levels,
        options_.blur_input_image);

  const size_t num_detected = feat.getNumFeatures();
  keypoints.reset(new OrbKeypointWrapper(num_detected));
  feat.getX().host(keypoints->x.get());
  feat.getY().host(keypoints->y.get());
  feat.getScore().host(keypoints->score.get());
  feat.getOrientation().host(keypoints->orient.get());
  feat.getSize().host(keypoints->size.get());
  desc.host(keypoints->descr.get());

  return num_detected;
}

} // ze namespace
