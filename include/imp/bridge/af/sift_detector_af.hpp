#pragma once

#include <imp/features/feature_detector.h>
#include <imp/bridge/af/image_af.hpp>

namespace ze {

struct SiftDetectorOptions
{

};

class SiftDetectorAF : public AbstractDetector
{
public:
  virtual ~SiftDetectorAF() = default;
  SiftDetectorAF(const SiftDetectorOptions& options, const Size2u& image_size);
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;
  virtual uint32_t detect(const ImageAF32fC1& im, KeypointsWrapper& keypoints);

private:
  SiftDetectorOptions options_;
};

} // ze namespace
