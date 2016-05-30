#pragma once

#include <imp/features/feature_detector.h>
#include <imp/bridge/af/image_af.hpp>

namespace ze {

struct OrbDetectorOptions
{
  float fast_thr{20.0f};
  uint16_t max_feat{400};
  float scale_factor{1.5};
  uint8_t levels{4};
  bool blur_img{false};
};

struct OrbKeypointWrapper
{
  static constexpr uint8_t kDescrLength{8};
  using Ptr = typename std::shared_ptr<OrbKeypointWrapper>;
  using OrbDescriptors = Eigen::Matrix<unsigned, kDescrLength, Eigen::Dynamic, Eigen::RowMajor>;
  OrbKeypointWrapper(uint32_t num)
    : num_detected(num)
  {
    x.reset(new float[num_detected]);
    y.reset(new float[num_detected]);
    score.reset(new float[num_detected]);
    orient.reset(new float[num_detected]);
    size.reset(new float[num_detected]);
    descr.reset(new unsigned[kDescrLength*num_detected]);
  }
  OrbDescriptors getDescriptors()
  {
    return Eigen::Map<OrbDescriptors>(
          descr.get(), kDescrLength, num_detected);
  }
  uint32_t num_detected{0};
  std::unique_ptr<float[]> x;
  std::unique_ptr<float[]> y;
  std::unique_ptr<float[]> score;
  std::unique_ptr<float[]> orient;
  std::unique_ptr<float[]> size;
  std::unique_ptr<unsigned[]> descr;
};

class OrbDetectorAF : public AbstractDetector
{
public:
  virtual ~OrbDetectorAF() = default;
  OrbDetectorAF(const OrbDetectorOptions& options, const Size2u& image_size);
  virtual uint32_t detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints) override;
  virtual uint32_t detect(const ImageAF32fC1& im, OrbKeypointWrapper::Ptr& keypoints);

private:
  OrbDetectorOptions options_;
};

} // ze namespace
