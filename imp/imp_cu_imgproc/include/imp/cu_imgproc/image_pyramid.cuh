#pragma once

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_reduce.cuh>

namespace ze {

/**
 * @brief The ImagePyramid class holds an image scale pyramid
 *
 * @todo (MWE) no roi support yet (e.g. propagated automatically from finest to coarser level)
 */
template<typename Pixel>
class ImagePyramid
{
public:
  ZE_POINTER_TYPEDEFS(ImagePyramid);

  // typedefs for convenience
  using Image = typename ze::Image<Pixel>;
  using ImagePtr = typename ze::ImagePtr<Pixel>;
  using ImageLevels = std::vector<ImagePtr>;

public:
  ImagePyramid() = delete;
  virtual ~ImagePyramid() = default;

  /**
   * @brief ImagePyramid constructs an empy image pyramid
   * @param size Image size of level 0
   * @param scale_factor multiplicative level-to-level scale factor
   * @param size_bound_ minimum size of the shorter side on coarsest level
   * @param max_num_levels maximum number of levels
   */
  ImagePyramid(Size2u size, float scale_factor=0.5f, uint32_t size_bound=8,
               uint32_t max_num_levels=UINT32_MAX);

  /** Clearing image pyramid, not resetting parameters though. */
  void clear() noexcept;

  /** Setting up levels. */
  void init(const ze::Size2u& size);

  /*
   * Getters / Setters
   */

  /** Returns the image pyramid (all levels) */
  inline ImageLevels& levels() {return levels_;}
  inline const ImageLevels& levels() const {return levels_;}

  /** Returns the actual number of levels saved in the pyramid. */
  inline size_t numLevels() const {return num_levels_;}

  /** Returns a reference to the \a i-th image of the pyramid level. */
  inline Image& operator[] (size_t i) {return *levels_[i];}
  inline const Image& operator[] (size_t i) const {return *levels_[i];}

  /** Returns a shared pointer to the \a i-th image of the pyramid level. */
  inline ImagePtr atShared(size_t i) {return levels_.at(i);}
  inline const ImagePtr atShared(size_t i) const {return levels_.at(i);}

  /** Returns a reference to i-th image of the pyramid level. */
  inline Image& at(size_t i) { return *levels_.at(i); }
  inline const Image& at(size_t i) const { return *levels_.at(i); }

  /** Returns the size of the i-th image. */
  inline Size2u size(size_t i) const {return this->at(i).size();}

  /** Sets the multiplicative level-to-level scale factor
   *  (most likely in the interval [0.5,1.0[)
   */
  inline void setScaleFactor(const float& scale_factor) {scale_factor_ = scale_factor;}
  /** Returns the multiplicative level-to-level scale factor. */
  inline float scaleFactor() const {return scale_factor_;}

  /** Returns the multiplicative scale-factor from \a i-th level to 0-level. */
  inline float scaleFactor(const size_t i) const
  {
    CHECK_LT(i, scale_factors_.size());
    return scale_factors_[i];
  }

  /** Sets the user defined maximum number of pyramid levels. */
  inline void setMaxNumLevels(const size_t max_num_levels)
  {
    max_num_levels_ = max_num_levels;
  }
  /** Returns the user defined maximum number of pyramid levels. */
  inline size_t maxNumLevels() const {return max_num_levels_;}


  /** Sets the user defined size bound for the coarsest level (short side). */
  inline void sizeBound(const uint32_t size_bound) {size_bound_ = size_bound;}
  /** Returns the user defined size bound for the coarsest level (short side). */
  inline uint32_t sizeBound() const {return size_bound_;}

  /** Factory function: Add image */
  inline void push_back(const ImagePtr& img) { levels_.push_back(img); }

  /** Perfect forwarding of the initialization. Avoids copying */
  template<typename... Args>
  void emplace_back(Args&&... args) { levels_.emplace_back(std::forward<Args>(args)...); }


private:


private:
  ImageLevels levels_; //!< Image pyramid levels holding shared_ptrs to images.
  std::vector<float> scale_factors_; //!< Scale factors (multiplicative) towards the 0-level.
  float scale_factor_ = 0.5f; //!< Scale factor between pyramid levels
  uint32_t size_bound_ = 8; //!< User defined minimum size of coarsest level (short side).
  size_t max_num_levels_ = UINT32_MAX; //!< User defined maximum number of pyramid levels.
  size_t num_levels_ = UINT32_MAX; //!< actual number of levels dependent on the current setting.
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImagePyramid<ze::Pixel8uC1> ImagePyramid8uC1;
typedef ImagePyramid<ze::Pixel8uC2> ImagePyramid8uC2;
typedef ImagePyramid<ze::Pixel8uC3> ImagePyramid8uC3;
typedef ImagePyramid<ze::Pixel8uC4> ImagePyramid8uC4;

typedef ImagePyramid<ze::Pixel16uC1> ImagePyramid16uC1;
typedef ImagePyramid<ze::Pixel16uC2> ImagePyramid16uC2;
typedef ImagePyramid<ze::Pixel16uC3> ImagePyramid16uC3;
typedef ImagePyramid<ze::Pixel16uC4> ImagePyramid16uC4;

typedef ImagePyramid<ze::Pixel32sC1> ImagePyramid32sC1;
typedef ImagePyramid<ze::Pixel32sC2> ImagePyramid32sC2;
typedef ImagePyramid<ze::Pixel32sC3> ImagePyramid32sC3;
typedef ImagePyramid<ze::Pixel32sC4> ImagePyramid32sC4;

typedef ImagePyramid<ze::Pixel32fC1> ImagePyramid32fC1;
typedef ImagePyramid<ze::Pixel32fC2> ImagePyramid32fC2;
typedef ImagePyramid<ze::Pixel32fC3> ImagePyramid32fC3;
typedef ImagePyramid<ze::Pixel32fC4> ImagePyramid32fC4;

//------------------------------------------------------------------------------
namespace cu {

//! Image Pyramid Factory:
template<typename Pixel>
std::shared_ptr<ImagePyramid<Pixel>>
createImagePyramidGpu(
    typename Image<Pixel>::Ptr img_level0, real_t scale_factor=0.5,
    uint32_t max_num_levels=UINT32_MAX, uint32_t size_bound=8u)
{
  // Sanity checks.
  CHECK(img_level0->isGpuMemory());
  using Pyr = ImagePyramid<Pixel>;
  auto pyr = std::make_shared<Pyr>(
        img_level0->size(), scale_factor, size_bound, max_num_levels);

  pyr->push_back(img_level0);
  Size2u sz0 =  img_level0->size();

  for (size_t i=1; i<pyr->numLevels(); ++i)
  {
    Size2u sz(static_cast<uint32_t>(sz0.width() * pyr->scaleFactor(i) + 0.5f),
              static_cast<uint32_t>(sz0.height() * pyr->scaleFactor(i) + 0.5f));

    // init level memory with either ImageGpu or ImageRaw
    using ImageGpu = typename ze::cu::ImageGpu<Pixel>;
    pyr->emplace_back(std::make_shared<ImageGpu>(sz));
    typename ImageGpu::Ptr prev = std::dynamic_pointer_cast<ImageGpu>(pyr->atShared(i-1));
    typename ImageGpu::Ptr img = std::dynamic_pointer_cast<ImageGpu>(pyr->atShared(i));
    VLOG(300) << "Creating GPU ImagePyramid Level " << i << " of size " << sz;
    ze::cu::reduce(*img, *prev, InterpolationMode::Linear, true);
  }
  return pyr;
}
} // namespace cu
} // namespace ze
