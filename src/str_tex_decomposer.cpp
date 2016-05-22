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
}


//-----------------------------------------------------------------------------
template<typename Pixel>
void StrTexDecomposer<Pixel>::solve(
    const ze::cu::ImageGpuPtr<Pixel>& src,
    const ze::cu::ImageGpuPtr<Pixel>& tex_image,
    const ze::cu::ImageGpuPtr<Pixel>& structure_image)
{

}


//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class StrTexDecomposer<ze::Pixel8uC1>;
template class StrTexDecomposer<ze::Pixel32fC1>;


} // namespace cu
} // namespace ze
