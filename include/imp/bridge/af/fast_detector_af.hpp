#pragma once

#include <imp/bridge/af/pyramid_af.hpp>
#include <imp/features/fast_detector.h>

namespace ze {

class FastDetectorAF : public FastDetector
{

public:
  virtual ~FastDetectorAF() = default;
  FastDetectorAF(const FastDetectorOptions& options, const Size2u& image_size);
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;
};

} // ze namespace

