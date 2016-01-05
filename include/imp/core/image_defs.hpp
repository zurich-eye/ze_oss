#ifndef IMP_IMAGE_DEFS_HPP
#define IMP_IMAGE_DEFS_HPP

#include <memory>
#include <imp/core/pixel_enums.hpp>

namespace imp {

template<typename Pixel, imp::PixelType pixel_type>
class ImageCv;

template<typename Pixel, imp::PixelType pixel_type>
using ImageCvPtr = typename std::shared_ptr<ImageCv<Pixel,pixel_type>>;

template<typename Pixel, imp::PixelType pixel_type>
using ConstImageCvPtrRef = const std::shared_ptr<ImageCv<Pixel,pixel_type>>&;



} // namespace imp

#endif // IMP_IMAGE_DEFS_HPP

