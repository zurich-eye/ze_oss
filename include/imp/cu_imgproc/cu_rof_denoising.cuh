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
class RofDenoising  : public imp::cu::VariationalDenoising
{
public:
  using Base = VariationalDenoising;
  using ImageGpu = imp::cu::ImageGpu<Pixel, pixel_type>;
  using Ptr = std::shared_ptr<RofDenoising<Pixel,pixel_type>>;

public:
  RofDenoising() = default;
  virtual ~RofDenoising() = default;
  using Base::Base;

  virtual void init(const Size2u& size) override;
  virtual void denoise(const std::shared_ptr<imp::ImageBase>& dst,
                       const std::shared_ptr<imp::ImageBase>& src) override;

  void primalDualEnergy(double& primal_energy, double& dual_energy);

protected:
  virtual void print(std::ostream &os) const override;

private:
  typename ImageGpu::Ptr f_;

  // pixel-wise primal and dual energies to avoid allocation of memory for every check
  std::unique_ptr<ImageGpu32fC1> primal_energies_;
  std::unique_ptr<ImageGpu32fC1> dual_energies_;

};

//-----------------------------------------------------------------------------
// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef RofDenoising<imp::Pixel8uC1, imp::PixelType::i8uC1> RofDenoising8uC1;
typedef RofDenoising<imp::Pixel32fC1, imp::PixelType::i32fC1> RofDenoising32fC1;

} // namespace cu
} // namespace imp

#endif // IMP_CU_ROF_DENOISING_CUH
