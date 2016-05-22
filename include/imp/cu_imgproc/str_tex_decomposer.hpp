#pragma once

#include <imp/cu_imgproc/cu_variational_denoising.cuh>

#include <memory>
#include <cuda_runtime_api.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/common/macros.h>

namespace ze {
namespace cu {

/** Decomposes the input image into its structure (cartoon) and texture part. */
template<typename Pixel>
class StrTexDecomposer
{
public:
  ZE_POINTER_TYPEDEFS(StrTexDecomposer);
  using Base = VariationalDenoising;
  using ImageGpu = ze::cu::ImageGpu<Pixel>;

public:
  StrTexDecomposer();
  ~StrTexDecomposer() = default;

  void solve(const ze::cu::ImageGpuPtr<Pixel>& src,
             const ze::cu::ImageGpuPtr<Pixel>& tex_image,
             const ze::cu::ImageGpuPtr<Pixel>& structure_image = NULL);

protected:
  void init(const Size2u& size);
//  void print(std::ostream &os) const;

private:
  ImageGpuPtr<Pixel> src_;
  ImageGpuPtr<Pixel> denoised_;
  ze::cu::VariationalDenoising::Ptr denoiser_;
};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef StrTexDecomposer<ze::Pixel8uC1> StrTexDecomposer8uC1;
typedef StrTexDecomposer<ze::Pixel32fC1> StrTexDecomposer32fC1;

template <typename Pixel>
using StrTexDecomposerPtr = typename std::shared_ptr<StrTexDecomposer<Pixel>>;

} // namespace cu
} // namespace ze
