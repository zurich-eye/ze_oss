#include <imp/cu_correspondence/variational_epipolar_stereo.hpp>

#include <imp/cu_correspondence/stereo_ctf_warping.hpp>

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
VariationalEpipolarStereo::VariationalEpipolarStereo(Parameters::Ptr params)
  : VariationalStereo(params)
{ ; }

//------------------------------------------------------------------------------
VariationalEpipolarStereo::~VariationalEpipolarStereo()
{ ; }

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setFundamentalMatrix(const cu::Matrix3f& F)
{
  ctf_->setFundamentalMatrix(F);
}


//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setIntrinsics(const Cameras& cams)
{
  ctf_->setIntrinsics(cams);
}

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setExtrinsics(const cu::SE3<float>& T_mov_fix)
{
  ctf_->setExtrinsics(T_mov_fix);
}

//------------------------------------------------------------------------------
void VariationalEpipolarStereo::setDepthProposal(
    const ImageGpu32fC1::Ptr& depth_proposal,
    const ImageGpu32fC1::Ptr& depth_proposal_sigma2)
{
  ctf_->setDepthProposal(depth_proposal, depth_proposal_sigma2);
}


} // namespace cu
} // namespace imp

