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

struct SiftKeypointWrapper
{
  static constexpr uint8_t kDescrLength{128};
  using Ptr = typename std::shared_ptr<SiftKeypointWrapper>;
  using Descriptors = Eigen::Matrix<float, kDescrLength, Eigen::Dynamic, Eigen::ColMajor>;
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
  Descriptors getDescriptors()
  {
    Descriptors res;
    for (uint32_t d=0; d<num_detected; ++d)
    {
      for (uint8_t i=0; i<kDescrLength; ++i)
      {
        res(i, d) = descr.get()[d][i];
      }
    }
    return res;
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
