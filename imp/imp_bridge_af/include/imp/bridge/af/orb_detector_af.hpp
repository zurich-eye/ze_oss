#pragma once

#include <imp/features/feature_detector.hpp>
#include <imp/bridge/af/image_af.hpp>

namespace ze {

using OrbDescriptors = Eigen::Matrix<unsigned, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

struct OrbDetectorOptions
{
  //! FAST threshold for which a pixel of the circle around the
  //! central pixel is considered to be brighter or darker
  float fast_threshold{20.0f};
  //! Maximum number of features to hold
  //! (will only keep the max_feat features with higher Harris responses)
  uint16_t max_num_features{400};
  //! Image downsample factor in interval (1, 2].
  //! A value of 2 corresponds to half-sampling the input image
  float scale_factor{1.5};
  //! Number of pyramid levels for feature esxtraction
  uint8_t pyramid_levels{4};
  //! Gaussian blur input image with sigma=2
  bool blur_input_image{false};
};

struct OrbKeypointWrapper
{
  static constexpr uint8_t c_descriptor_length{8};
  using Ptr = typename std::shared_ptr<OrbKeypointWrapper>;

  void allocate(uint32_t num)
  {
    num_detected = num;
    x.reset(new float[num_detected]);
    y.reset(new float[num_detected]);
    score.reset(new float[num_detected]);
    orient.reset(new float[num_detected]);
    size.reset(new float[num_detected]);
    descr.reset(new unsigned[c_descriptor_length*num_detected]);
  }

  inline Keypoints getKeypoints() const
  {
    Keypoints keypoints(2, num_detected);
    for (size_t k = 0; k < num_detected; ++k)
    {
      keypoints(0, k) = x.get()[k];
      keypoints(1, k) = y.get()[k];
    }
    return keypoints;
  }

  inline KeypointScores getKeypointScores() const
  {
    KeypointScores scores(num_detected);
    for (size_t k = 0; k < num_detected; ++k)
    {
      scores(k) = score.get()[k];
    }
    return scores;
  }

  inline KeypointSizes getKeypointSizes() const
  {
    KeypointSizes sizes(num_detected);
    for (size_t k = 0; k < num_detected; ++k)
    {
      sizes(k) = size.get()[k];
    }
    return sizes;
  }

  inline KeypointAngles getKeypointAngles() const
  {
    KeypointAngles angles(num_detected);
    for (size_t k = 0; k < num_detected; ++k)
    {
      angles(k) = orient.get()[k];
    }
    return angles;
  }

  inline OrbDescriptors getDescriptors() const
  {
    OrbDescriptors descriptors(c_descriptor_length, num_detected);
    for (size_t k = 0; k < num_detected; ++k)
    {
      for (uint8_t d = 0; d < c_descriptor_length; ++d)
      {
        descriptors(d, k) = descr.get()[k*c_descriptor_length+d];
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
  virtual uint32_t detect(const ImageAF32fC1& im, OrbKeypointWrapper& keypoints);

private:
  OrbDetectorOptions options_;
};

} // ze namespace
