#ifndef IMP_CU_OPENCV_BRIDGE_HPP
#define IMP_CU_OPENCV_BRIDGE_HPP

#include <memory>

#include <imp/bridge/opencv/cv_bridge.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>


namespace imp {
namespace cu {

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void cvBridgeLoad(imp::cu::ImageGpuPtr<Pixel,pixel_type>& out, const std::string& filename,
                  imp::PixelOrder pixel_order)
{
  ImageCvPtr<Pixel,pixel_type> cv_img;
  imp::cvBridgeLoad<Pixel,pixel_type>(cv_img, filename, pixel_order);
  out = std::make_shared<imp::cu::ImageGpu<Pixel,pixel_type>>(*cv_img);
}

//------------------------------------------------------------------------------
template<typename Pixel, imp::PixelType pixel_type>
void cvBridgeShow(const std::string& winname,
                  const imp::cu::ImageGpu<Pixel,pixel_type>& img, bool normalize=false)
{
  const ImageCv<Pixel, pixel_type> cv_img(img);
  imp::cvBridgeShow(winname, cv_img, normalize);
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T, imp::PixelType pixel_type>
void cvBridgeShow(const std::string& winname,
                  const imp::cu::ImageGpu<Pixel, pixel_type>& img,
                  const T& min, const T& max)
{
  const ImageCv<Pixel, pixel_type> cv_img(img);
  imp::cvBridgeShow(winname, cv_img, min, max);
}

} // namespace cu
} // namespace imp

#endif // IMP_CU_OPENCV_BRIDGE_HPP
