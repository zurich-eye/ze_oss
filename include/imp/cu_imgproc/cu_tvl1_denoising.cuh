#ifndef IMP_CU_ROF_DENOISING_CUH
#define IMP_CU_ROF_DENOISING_CUH

#include <imp/cu_imgproc/cu_variational_denoising.cuh>

#include <memory>
#include <cuda_runtime_api.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>

namespace imp {
namespace cu {

template<typename Pixel, imp::PixelType pixel_type>
class TvL1Denoising  : public imp::cu::VariationalDenoising
{
public:
  using Ptr = std::shared_ptr<TvL1Denoising<Pixel,pixel_type>>;
  using Base = VariationalDenoising;
  using ImageGpu = imp::cu::ImageGpu<Pixel, pixel_type>;

public:
  TvL1Denoising() = default;
  virtual ~TvL1Denoising() = default;
  using Base::Base;

  virtual void init(const Size2u& size) override;
  virtual void __host__ denoise(const ImageBase::Ptr& dst,
                                const ImageBase::Ptr& src) override;

protected:
  virtual void print(std::ostream &os) const override;

private:
  typename ImageGpu::Ptr f_;

};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef TvL1Denoising<imp::Pixel8uC1, imp::PixelType::i8uC1> TvL1Denoising8uC1;
typedef TvL1Denoising<imp::Pixel32fC1, imp::PixelType::i32fC1> TvL1Denoising32fC1;

} // namespace cu
} // namespace imp

#endif // IMP_CU_ROF_DENOISING_CUH
