#include <imp/cu_imgproc/str_tex_decomposer.hpp>

#include <imp/cu_imgproc/cu_rof_denoising.cuh>
#include <ze/common/logging.hpp>

namespace ze {
namespace cu {
//-----------------------------------------------------------------------------
template<typename Pixel>
StrTexDecomposer<Pixel>::StrTexDecomposer()
  : denoiser_(new RofDenoising<Pixel>())
{

}

//-----------------------------------------------------------------------------
template<typename Pixel>
void StrTexDecomposer<Pixel>::init(const Size2u& size)
{
  size_ = size;
  denoised_ = std::make_shared<ze::cu::ImageGpu<Pixel>>(size_);
}


//-----------------------------------------------------------------------------
template<typename Pixel>
void StrTexDecomposer<Pixel>::solve(
    const ze::cu::ImageGpuPtr<Pixel>& src,
    const ze::cu::ImageGpuPtr<Pixel>& tex_image,
    const ze::cu::ImageGpuPtr<Pixel>& structure_image)
{
  CHECK(src);
  CHECK(tex_image);

  CHECK_EQ(src->size(), tex_image->size());
  if (structure_image)
  {
    CHECK_EQ(src->size(), structure_image->size());
  }

  if (src->size() != size_ || !denoised_)
  {
    this->init(src->size());
  }

  src_ = src;
  tex_ = tex_image;
  str_ = (structure_image) ? structure_image :
                             std::make_shared<ze::cu::ImageGpu<Pixel>>(size_);

  denoiser_->params().lambda = 1.0f;
  denoiser_->params().max_iter = 100;
  denoiser_->denoise(denoised_, src_);

  //! @todo (MWE) implement addWeighted!
}


//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class StrTexDecomposer<ze::Pixel8uC1>;
template class StrTexDecomposer<ze::Pixel32fC1>;


} // namespace cu
} // namespace ze
