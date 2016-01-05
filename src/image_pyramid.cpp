#include <imp/cu_imgproc/image_pyramid.hpp>

#include <imp/core/exception.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/image.hpp>
#include <imp/core/image_raw.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/cu_reduce.cuh>


namespace imp {


//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
ImagePyramid<Pixel,pixel_type>::ImagePyramid(ImagePtr img, float scale_factor,
                                             std::uint32_t size_bound,
                                             size_t max_num_levels)
  : scale_factor_(scale_factor)
  , size_bound_(size_bound)
  , max_num_levels_(max_num_levels)
{
  this->init(img->size());
  this->updateImage(img, imp::InterpolationMode::linear);
}

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void ImagePyramid<Pixel,pixel_type>::clear() noexcept
{
  levels_.clear();
  scale_factors_.clear();
}

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void ImagePyramid<Pixel,pixel_type>::init(const imp::Size2u& size)
{
  if (scale_factor_<=0.0 || scale_factor_>=1)
  {
    throw imp::Exception("Initializing image pyramid with scale factor <=0 or >=1 not possible.",
                         __FILE__, __FUNCTION__, __LINE__);
  }

  if (!levels_.empty() || scale_factors_.empty())
  {
    this->clear();
  }

  std::uint32_t shorter_side = std::min(size.width(), size.height());

  // calculate the maximum number of levels
  float ratio = static_cast<float>(shorter_side)/static_cast<float>(size_bound_);
  // +1 because the original size is level 0
  size_t possible_num_levels =
      static_cast<int>(-std::log(ratio)/std::log(scale_factor_)) + 1;
  num_levels_ = std::min(max_num_levels_, possible_num_levels);

  // init rate for each level
  for (size_t i = 0; i<num_levels_; ++i)
  {
    scale_factors_.push_back(std::pow(scale_factor_, static_cast<float>(i)));
  }
}

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void ImagePyramid<Pixel,pixel_type>::updateImage(ImagePtr img_level0,
                                                 InterpolationMode interp)
{
  // TODO sanity checks

  levels_.push_back(img_level0);
  Size2u sz0 =  img_level0->size();

  for (size_t i=1; i<num_levels_; ++i)
  {
    Size2u sz(static_cast<std::uint32_t>(sz0.width()*scale_factors_[i] + 0.5f),
              static_cast<std::uint32_t>(sz0.height()*scale_factors_[i] + 0.5f));

    // init level memory with either ImageGpu or ImageRaw
    if(img_level0->isGpuMemory())
    {
      using ImageGpu = typename imp::cu::ImageGpu<Pixel,pixel_type>;
      typename ImageGpu::Ptr img = std::make_shared<ImageGpu>(sz);
      typename ImageGpu::Ptr prev = std::dynamic_pointer_cast<ImageGpu>(levels_.back());
      imp::cu::reduce(*img, *prev, interp, true);
      levels_.push_back(img);
    }
    else
    {
      using ImageRaw = imp::ImageRaw<Pixel,pixel_type>;
      typename ImageRaw::Ptr img = std::make_shared<ImageRaw>(sz);
      //! @todo (MWE) cpu reduction
      throw imp::Exception("CPU reduction not yet implemented.", __FILE__, __FUNCTION__, __LINE__);
      levels_.push_back(img);
    }
  }

}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImagePyramid<imp::Pixel8uC1, imp::PixelType::i8uC1>;
template class ImagePyramid<imp::Pixel8uC2, imp::PixelType::i8uC2>;
//template class ImagePyramid<imp::Pixel8uC3, imp::PixelType::i8uC3>;
template class ImagePyramid<imp::Pixel8uC4, imp::PixelType::i8uC4>;

template class ImagePyramid<imp::Pixel16uC1, imp::PixelType::i16uC1>;
template class ImagePyramid<imp::Pixel16uC2, imp::PixelType::i16uC2>;
//template class ImagePyramid<imp::Pixel16uC3, imp::PixelType::i16uC3>;
template class ImagePyramid<imp::Pixel16uC4, imp::PixelType::i16uC4>;

template class ImagePyramid<imp::Pixel32sC1, imp::PixelType::i32sC1>;
template class ImagePyramid<imp::Pixel32sC2, imp::PixelType::i32sC2>;
//template class ImagePyramid<imp::Pixel32sC3, imp::PixelType::i32sC3>;
template class ImagePyramid<imp::Pixel32sC4, imp::PixelType::i32sC4>;

template class ImagePyramid<imp::Pixel32fC1, imp::PixelType::i32fC1>;
template class ImagePyramid<imp::Pixel32fC2, imp::PixelType::i32fC2>;
//template class ImagePyramid<imp::Pixel32fC3, imp::PixelType::i32fC3>;
template class ImagePyramid<imp::Pixel32fC4, imp::PixelType::i32fC4>;


} // namespace imp

