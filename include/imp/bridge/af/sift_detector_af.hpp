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
  float intensity_scale{1.0f};
  float feature_ratio{0.05f};
};

struct SiftKeypointWrapper
{
  static constexpr uint8_t kDescrLength{128};
  using Ptr = typename std::shared_ptr<SiftKeypointWrapper>;
  using Descriptors = std::vector<float[kDescrLength]>; //! TODO (MPI) what container do we want for the descriptors?
  SiftKeypointWrapper(uint32_t num)
    : num_detected(num)
  {
    x.reset(new float[num]);
    y.reset(new float[num]);
    score.reset(new float[num]);
    orient.reset(new float[num]);
    size.reset(new float[num]);
    descr.reset(new float[num][kDescrLength]);
  }
  uint32_t num_detected{0};
  std::unique_ptr<float[]> x;
  std::unique_ptr<float[]> y;
  std::unique_ptr<float[]> score;
  std::unique_ptr<float[]> orient;
  std::unique_ptr<float[]> size;
  std::unique_ptr<float[kDescrLength]> descr;
};

class SiftDetectorAF : public AbstractDetector
{
public:
  virtual ~SiftDetectorAF() = default;
  SiftDetectorAF(const SiftDetectorOptions& options, const Size2u& image_size);
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;
  virtual uint32_t detect(const ImageAF32fC1& im, SiftKeypointWrapper::Ptr& keypoints);

private:
  SiftDetectorOptions options_;
};

} // ze namespace
