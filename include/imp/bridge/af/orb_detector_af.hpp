#pragma once

#include <imp/features/feature_detector.h>
#include <imp/bridge/af/image_af.hpp>

namespace ze {

using OrbDescriptors = Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

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

  inline Keypoints getKeypoints() const
  {
    Keypoints keypoints(2, num_detected);
    for(size_t k=0; k<num_detected; ++k)
    {
      keypoints(0, k) = x.get()[k];
      keypoints(1, k) = y.get()[k];
    }
    return keypoints;
  }

  inline KeypointScores getKeypointScores() const
  {
    KeypointScores scores(num_detected);
    for(size_t k=0; k<num_detected; ++k)
    {
      scores(k) = score.get()[k];
    }
    return scores;
  }

  inline KeypointSizes getKeypointSizes() const
  {
    KeypointSizes sizes(num_detected);
    for(size_t k=0; k<num_detected; ++k)
    {
      sizes(k) = size.get()[k];
    }
    return sizes;
  }

  inline KeypointAngles getKeypointAngles() const
  {
    KeypointAngles angles(num_detected);
    for(size_t k=0; k<num_detected; ++k)
    {
      angles(k) = orient.get()[k];
    }
    return angles;
  }

  inline OrbDescriptors getDescriptors() const
  {
    OrbDescriptors descriptors(kDescrLength, num_detected);
    for(size_t k=0; k<num_detected; ++k)
    {
      for(uint8_t d=0; d<kDescrLength; ++d)
      {
        descriptors(d, k) = descr.get()[k*kDescrLength+d];
      }
    }
    return descriptors;
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
