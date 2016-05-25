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

uint32_t SiftDetectorAF::detect(const ImageAF32fC1& im, SiftKeypointWrapper::Ptr& keypoints)
{
  af::features feat;
  af::array desc;
  af::sift(
        feat,
        desc,
        im.afArray(),
        options_.num_layers,
        options_.contrast_thr,
        options_.edge_thr,
        options_.init_sigma,
        options_.double_input,
        options_.intensity_scale,
        options_.feature_ratio);

  const size_t num_detected = feat.getNumFeatures();
  keypoints.reset(new SiftKeypointWrapper(num_detected));
  feat.getX().host(keypoints->x.get());
  feat.getY().host(keypoints->y.get());
  feat.getScore().host(keypoints->score.get());
  feat.getOrientation().host(keypoints->orient.get());
  feat.getSize().host(keypoints->size.get());
  desc.host(keypoints->descr.get());

  return num_detected;
}

} // ze namespace
