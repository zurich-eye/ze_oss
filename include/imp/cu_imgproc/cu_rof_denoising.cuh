#ifndef IMP_CU_ROF_DENOISING_CUH
#define IMP_CU_ROF_DENOISING_CUH

#include <imp/cu_imgproc/cu_variational_denoising.cuh>

#include <memory>
#include <cuda_runtime_api.h>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_utils.hpp>
#include <ze/common/macros.h>

namespace ze {
namespace cu {

template<typename Pixel>
class RofDenoising  : public ze::cu::VariationalDenoising
{
public:
  ZE_POINTER_TYPEDEFS(RofDenoising);
  using Base = VariationalDenoising;
  using ImageGpu = ze::cu::ImageGpu<Pixel>;

public:
  RofDenoising() = default;
  virtual ~RofDenoising() = default;
  using Base::Base;

  virtual void denoise(const ze::ImageBase::Ptr& dst,
                       const ze::ImageBase::Ptr& src) override;

  void primalDualEnergy(double& primal_energy, double& dual_energy);

protected:
  virtual void init(const Size2u& size) override;
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
typedef RofDenoising<ze::Pixel8uC1> RofDenoising8uC1;
typedef RofDenoising<ze::Pixel32fC1> RofDenoising32fC1;

template <typename Pixel>
using RofDenoisingPtr = typename std::shared_ptr<RofDenoising<Pixel>>;

} // namespace cu
} // namespace ze

#endif // IMP_CU_ROF_DENOISING_CUH
