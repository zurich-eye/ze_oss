#include <imp/cu_imgproc/cu_variational_denoising.cuh>
#include <imp/cu_core/cu_texture.cuh>

namespace imp {
namespace cu {

//-----------------------------------------------------------------------------
VariationalDenoising::VariationalDenoising()
  : f_tex_(nullptr)
  , u_tex_(nullptr)
  , u_prev_tex_(nullptr)
  , p_tex_(nullptr)
{
}

//-----------------------------------------------------------------------------
VariationalDenoising::~VariationalDenoising()
{
}

//-----------------------------------------------------------------------------
__host__ void VariationalDenoising::init(const Size2u& size)
{
  size_ = size;
  fragmentation_.reset(new Fragmentation(size));

  // setup internal memory
  this->u_.reset(new ImageGpu32fC1(size));
  this->u_prev_.reset(new ImageGpu32fC1(size));
  this->p_.reset(new ImageGpu32fC2(size));
}


} // namespace cu
} // namespace imp

