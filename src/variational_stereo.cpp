#include <imp/cu_correspondence/variational_stereo.hpp>
#include <imp/cu_correspondence/stereo_ctf_warping.hpp>

namespace ze {
namespace cu {

//------------------------------------------------------------------------------
VariationalStereo::VariationalStereo(Parameters::Ptr params)
{
  if (params)
  {
    params_ = params;
  }
  else
  {
    params_ = std::make_shared<Parameters>();
  }

  ctf_.reset(new StereoCtFWarping(params_));
}


//------------------------------------------------------------------------------
VariationalStereo::~VariationalStereo()
{ ; }


//------------------------------------------------------------------------------
void VariationalStereo::addImage(const ImageGpu32fC1::Ptr& image)
{
  CHECK(image != nullptr) << "Invalid input image.";
  ctf_->addImage(image);
}

//------------------------------------------------------------------------------
void VariationalStereo::reset()
{
  ctf_->reset();
}

//------------------------------------------------------------------------------
void VariationalStereo::solve()
{
  ctf_->solve();
}

//------------------------------------------------------------------------------
ImageGpu32fC1::Ptr VariationalStereo::computePrimalEnergy(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->computePrimalEnergy(level);
}

//------------------------------------------------------------------------------
VariationalStereo::ImageGpu32fC1::Ptr VariationalStereo::getDisparities(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->getDisparities(level);
}


//------------------------------------------------------------------------------
VariationalStereo::ImageGpu32fC1::Ptr VariationalStereo::getOcclusion(size_t level)
{
  CHECK_LT(level, params_->ctf.coarsest_level);
  return ctf_->getOcclusion(level);
}


} // namespace cu
} // namespace ze

