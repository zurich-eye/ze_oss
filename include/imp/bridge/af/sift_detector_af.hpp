#pragma once

#include <imp/features/feature_detector.h>
#include <imp/bridge/af/image_af.hpp>

namespace ze {

struct SiftDetectorOptions
{
  uint8_t num_layers{3};
  float contrast_thr{0.04f};
  float edge_thr{10.0f};
  float init_sigma{1.6f};
  bool double_input{true};
  float intensity_scale{0.00390625f};
  float feature_ratio{0.05f};
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
