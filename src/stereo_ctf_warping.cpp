#include <imp/cu_correspondence/stereo_ctf_warping.hpp>

#include <memory>

#include <glog/logging.h>

#include <imp/cu_correspondence/solver_stereo_huber_l1.cuh>
#include <imp/cu_correspondence/solver_stereo_precond_huber_l1.cuh>
#include <imp/cu_correspondence/solver_stereo_precond_huber_l1_weighted.cuh>
#include <imp/cu_correspondence/solver_epipolar_stereo_precond_huber_l1.cuh>

#include <imp/cu_core/cu_utils.hpp>

namespace imp {
namespace cu {

//------------------------------------------------------------------------------
StereoCtFWarping::StereoCtFWarping(Parameters::Ptr params)
  : params_(params)
{
}

//------------------------------------------------------------------------------
StereoCtFWarping::~StereoCtFWarping()
{
  // thanks to managed ptrs
}

//------------------------------------------------------------------------------
void StereoCtFWarping::init()
{
  if (image_pyramids_.empty())
  {
    throw Exception("No Image set, can't initialize when number of levels is unknown.",
                    __FILE__, __FUNCTION__, __LINE__);
  }

  //  // just in case
  //  levels_.clear();

  for (size_t i=params_->ctf.finest_level; i<=params_->ctf.coarsest_level; ++i)
  {
    Size2u sz = image_pyramids_.front()->size(i);
    switch (params_->solver)
    {
    case StereoPDSolver::HuberL1:
      levels_.emplace_back(new SolverStereoHuberL1(params_, sz, i));
    break;
    case StereoPDSolver::PrecondHuberL1:
      levels_.emplace_back(new SolverStereoPrecondHuberL1(params_, sz, i));
    break;
    case StereoPDSolver::PrecondHuberL1Weighted:
      levels_.emplace_back(new SolverStereoPrecondHuberL1Weighted(params_, sz, i));
    break;
    case StereoPDSolver::EpipolarPrecondHuberL1:
    {
      if (!depth_proposal_)
      {
        depth_proposal_.reset(new ImageGpu32fC1(image_pyramids_.front()->size(0)));
        depth_proposal_->setValue(0.f);
      }
      if (!depth_proposal_sigma2_)
      {
        depth_proposal_sigma2_.reset(new ImageGpu32fC1(image_pyramids_.front()->size(0)));
        depth_proposal_sigma2_->setValue(0.f);
      }

      levels_.emplace_back(new SolverEpipolarStereoPrecondHuberL1(
                             params_, sz, i, cams_, F_, T_mov_fix_,
                             *depth_proposal_, *depth_proposal_sigma2_));
    }
    break;
    }
  }
}

//------------------------------------------------------------------------------
bool StereoCtFWarping::ready()
{
  // check if all vectors are of the same length and not empty
  size_t desired_num_levels =
      params_->ctf.coarsest_level - params_->ctf.finest_level + 1;

  if (images_.empty() || image_pyramids_.empty() || levels_.empty() ||
      params_->ctf.coarsest_level < params_->ctf.finest_level ||
      images_.size() < 2 || // at least two images -> maybe adapt to the algorithm?
      image_pyramids_.front()->numLevels() < desired_num_levels ||
      levels_.size() < desired_num_levels)
  {
    return false;
  }
  return true;
}

//------------------------------------------------------------------------------
void StereoCtFWarping::addImage(const ImageGpu32fC1::Ptr& image)
{
  // generate image pyramid
  ImagePyramid32fC1::Ptr pyr(new ImagePyramid32fC1(image, params_->ctf.scale_factor, 4));

  // update number of levels
  if (params_->ctf.levels > pyr->numLevels())
    params_->ctf.levels = pyr->numLevels();
  if (params_->ctf.coarsest_level > params_->ctf.levels - 1)
    params_->ctf.coarsest_level = params_->ctf.levels - 1;

  images_.push_back(image);
  image_pyramids_.push_back(pyr);

  LOG(INFO) << "we have now " << images_.size() << " images and "
            <<  image_pyramids_.size() << " pyramids in the CTF instance. "
             << "params_->ctf.levels: " << params_->ctf.levels
             << " (" << params_->ctf.coarsest_level
             << " -> " << params_->ctf.finest_level << ")";
}

//------------------------------------------------------------------------------
void StereoCtFWarping::solve()
{
  if (levels_.empty())
  {
    this->init();
  }

  if (!this->ready())
  {
    throw imp::Exception("not initialized correctly. bailing out.",
                         __FILE__, __FUNCTION__, __LINE__);
  }

  // the image vector that is used as input for the level solvers
  std::vector<ImageGpu32fC1::Ptr> lev_images;


  // the first level is initialized differently so we solve this one first
  size_t lev = params_->ctf.coarsest_level;
  levels_.at(lev)->init();
  // gather images of current scale level
  lev_images.clear();
  for (auto pyr : image_pyramids_)
  {
    lev_images.push_back(std::dynamic_pointer_cast<ImageGpu32fC1>(pyr->at(lev)));
  }
  levels_.at(lev)->solve(lev_images);

  // and then loop until we reach the finest level
  // note that we loop with +1 idx as we would result in a buffer underflow
  // due to operator-- on size_t which is an unsigned type.
  for (; lev > params_->ctf.finest_level; --lev)
  {
    levels_.at(lev-1)->init(*levels_.at(lev));

    // gather images of current scale level
    lev_images.clear();
    for (auto pyr : image_pyramids_)
    {
      lev_images.push_back(std::dynamic_pointer_cast<ImageGpu32fC1>(pyr->at(lev-1)));
    }
    levels_.at(lev-1)->solve(lev_images);
  }
}

//------------------------------------------------------------------------------
ImageGpu32fC1::Ptr StereoCtFWarping::computePrimalEnergy(size_t level)
{
  if (!this->ready())
  {
    throw Exception("not initialized correctly; bailing out.",
                    __FILE__, __FUNCTION__, __LINE__);
  }
  level = max(params_->ctf.finest_level,
              min(params_->ctf.coarsest_level, level));
  return levels_.at(level)->computePrimalEnergy();
}

//------------------------------------------------------------------------------
StereoCtFWarping::ImageGpu32fC1::Ptr StereoCtFWarping::getDisparities(size_t level)
{
  if (!this->ready())
  {
    throw Exception("not initialized correctly; bailing out.",
                    __FILE__, __FUNCTION__, __LINE__);
  }
  level = max(params_->ctf.finest_level,
              min(params_->ctf.coarsest_level, level));
  return levels_.at(level)->getDisparities();
}

//------------------------------------------------------------------------------
StereoCtFWarping::ImageGpu32fC1::Ptr StereoCtFWarping::getOcclusion(size_t level)
{
  if (!this->ready())
  {
    throw Exception("not initialized correctly; bailing out.",
                    __FILE__, __FUNCTION__, __LINE__);
  }
  level = max(params_->ctf.finest_level,
              min(params_->ctf.coarsest_level, level));
  return levels_.at(level)->getOcclusion();
}

} // namespace cu
} // namespace imp

