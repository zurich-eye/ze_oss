#ifndef IMP_IMAGEPYRAMID_HPP
#define IMP_IMAGEPYRAMID_HPP

#include <vector>
#include <memory>

#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/pixel.hpp>
#include <imp/core/size.hpp>
#include <imp/core/image.hpp>

namespace imp {

/**
 * @brief The ImagePyramid class holds an image scale pyramid
 *
 * @todo (MWE) roi support (propagated automatically from finest to coarser level)
 */
template<typename Pixel, imp::PixelType pixel_type>
class ImagePyramid
{
public:
  using Ptr = std::shared_ptr<ImagePyramid>;

  // typedefs for convenience
  using Image = typename imp::Image<Pixel, pixel_type>;
  using ImagePtr = typename imp::ImagePtr<Pixel, pixel_type>;
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
//  ImagePyramid(const imp::Size2u& size, float scale_factor, std::uint32_t size_bound_=8,
//               size_t max_num_levels=UINT32_MAX);

  ImagePyramid(ImagePtr img, float scale_factor=0.5f, std::uint32_t size_bound_=8,
               size_t max_num_levels=UINT32_MAX);

//  ImagePyramid(ImagePtr img, float scale_factor, std::uint32_t size_bound_=8,
//               size_t max_num_levels=UINT32_MAX);


  /** Clearing image pyramid, not resetting parameters though. */
  void clear() noexcept;

  /** Setting up levels. */
  void init(const imp::Size2u& size);

  /** Initializing the images. */
  void updateImage(ImagePtr img_level0, InterpolationMode interp);

  /*
   * Getters / Setters
   */

  /** Returns the image pyramid (all levels) */
  inline ImageLevels& levels() {return levels_;}
  inline const ImageLevels& levels() const {return levels_;}

  /** Returns the actual number of levels saved in the pyramid. */
  inline size_t numLevels() {return num_levels_;}

  /** Returns a shared pointer to the \a i-th image of the pyramid level. */
  inline ImagePtr operator[] (size_t i) {return levels_[i];}
  inline const ImagePtr operator[] (size_t i) const {return levels_[i];}

  /** Returns a shared pointer to the \a i-th image of the pyramid level. */
  inline ImagePtr at(size_t i) {return levels_.at(i);}
  inline const ImagePtr at(size_t i) const {return levels_.at(i);}

  /** Returns the size of the i-th image. */
  inline Size2u size(size_t i) {return this->at(i)->size();}

  /** Sets the multiplicative level-to-level scale factor
   *  (most likely in the interval [0.5,1.0[)
   */
  inline void setScaleFactor(const float& scale_factor) {scale_factor_ = scale_factor;}
  /** Returns the multiplicative level-to-level scale factor. */
  inline float scaleFactor() const {return scale_factor_;}

  /** Returns the multiplicative scale-factor from \a i-th level to 0-level. */
  inline float scaleFactor(const size_t& i) const {return scale_factors_.at(i);}

  /** Sets the user defined maximum number of pyramid levels. */
  inline void setMaxNumLevels(const size_t& max_num_levels)
  {
    max_num_levels_ = max_num_levels;
  }
  /** Returns the user defined maximum number of pyramid levels. */
  inline size_t maxNumLevels() const {return max_num_levels_;}


  /** Sets the user defined size bound for the coarsest level (short side). */
  inline void sizeBound(const std::uint32_t& size_bound) {size_bound_ = size_bound;}
  /** Returns the user defined size bound for the coarsest level (short side). */
  inline std::uint32_t sizeBound() const {return size_bound_;}

private:


private:
  ImageLevels levels_; //!< Image pyramid levels holding shared_ptrs to images.
  std::vector<float> scale_factors_; //!< Scale factors (multiplicative) towards the 0-level.
  float scale_factor_ = 0.5f; //!< Scale factor between pyramid levels
  std::uint32_t size_bound_ = 8; //!< User defined minimum size of coarsest level (short side).
  size_t max_num_levels_ = UINT32_MAX; //!< User defined maximum number of pyramid levels.
  size_t num_levels_ = UINT32_MAX; //!< actual number of levels dependent on the current setting.
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef ImagePyramid<imp::Pixel8uC1, imp::PixelType::i8uC1> ImagePyramid8uC1;
typedef ImagePyramid<imp::Pixel8uC2, imp::PixelType::i8uC2> ImagePyramid8uC2;
typedef ImagePyramid<imp::Pixel8uC3, imp::PixelType::i8uC3> ImagePyramid8uC3;
typedef ImagePyramid<imp::Pixel8uC4, imp::PixelType::i8uC4> ImagePyramid8uC4;

typedef ImagePyramid<imp::Pixel16uC1, imp::PixelType::i16uC1> ImagePyramid16uC1;
typedef ImagePyramid<imp::Pixel16uC2, imp::PixelType::i16uC2> ImagePyramid16uC2;
typedef ImagePyramid<imp::Pixel16uC3, imp::PixelType::i16uC3> ImagePyramid16uC3;
typedef ImagePyramid<imp::Pixel16uC4, imp::PixelType::i16uC4> ImagePyramid16uC4;

typedef ImagePyramid<imp::Pixel32sC1, imp::PixelType::i32sC1> ImagePyramid32sC1;
typedef ImagePyramid<imp::Pixel32sC2, imp::PixelType::i32sC2> ImagePyramid32sC2;
typedef ImagePyramid<imp::Pixel32sC3, imp::PixelType::i32sC3> ImagePyramid32sC3;
typedef ImagePyramid<imp::Pixel32sC4, imp::PixelType::i32sC4> ImagePyramid32sC4;

typedef ImagePyramid<imp::Pixel32fC1, imp::PixelType::i32fC1> ImagePyramid32fC1;
typedef ImagePyramid<imp::Pixel32fC2, imp::PixelType::i32fC2> ImagePyramid32fC2;
typedef ImagePyramid<imp::Pixel32fC3, imp::PixelType::i32fC3> ImagePyramid32fC3;
typedef ImagePyramid<imp::Pixel32fC4, imp::PixelType::i32fC4> ImagePyramid32fC4;


} // namespace imp

#endif // IMP_IMAGEPYRAMID_HPP
