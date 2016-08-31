#include <imp/cu_imgproc/str_tex_decomposer.hpp>

#include <imp/cu_core/cu_math.cuh>
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
void StrTexDecomposer<Pixel>::solve(
    const ze::cu::ImageGpuPtr<Pixel>& tex_image,
    const ze::cu::ImageGpuPtr<Pixel>& src,
    const ze::cu::ImageGpuPtr<Pixel>& structure_image)
{
  CHECK(src);
  CHECK(tex_image);

  CHECK_EQ(src->size(), tex_image->size());
  if (structure_image)
  {
    CHECK_EQ(src->size(), structure_image->size());
  }

  src_ = src;
  tex_ = tex_image;
  // in-place denoising if no structure image desired for an output
  str_ = structure_image ? structure_image : tex_image;

  denoiser_->params().lambda = 1.0f;
  denoiser_->params().max_iter = 100;
  denoiser_->denoise(str_, src_);
  ze::cu::weightedSum(*tex_, *src_, weight_, *str_, 1.f-2.f*weight_);
}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class StrTexDecomposer<ze::Pixel8uC1>;
template class StrTexDecomposer<ze::Pixel32fC1>;


} // namespace cu
} // namespace ze
