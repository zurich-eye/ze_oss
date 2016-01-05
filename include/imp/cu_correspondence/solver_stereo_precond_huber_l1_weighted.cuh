#ifndef IMP_CU_STEREO_CTF_WARPING_LEVEL_PRECOND_HUBER_L1_WEIGHTED_CUH
#define IMP_CU_STEREO_CTF_WARPING_LEVEL_PRECOND_HUBER_L1_WEIGHTED_CUH

#include <cstdint>
#include <memory>

#include <imp/cu_correspondence/solver_stereo_abstract.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/core/size.hpp>

namespace imp {
namespace cu {

// forward decl
class Texture2D;

/**
 * @brief The StereoCtFWarpingLevelPrecondHuberL1 class
 * @todo better polymorphism! (derive from StereoCtfWarpingLevelPrecondHuberL1
 */
class SolverStereoPrecondHuberL1Weighted : public SolverStereoAbstract
{
public:
  SolverStereoPrecondHuberL1Weighted() = delete;
  virtual ~SolverStereoPrecondHuberL1Weighted();

  SolverStereoPrecondHuberL1Weighted(
      const Parameters::Ptr& params,
      imp::Size2u size, size_t level);

  virtual void init() override;
  virtual void init(const SolverStereoAbstract& rhs) override;
  virtual void solve(std::vector<ImageGpu32fC1::Ptr> images) override;

  virtual ImageGpu32fC1::Ptr computePrimalEnergy() override;

  virtual inline ImageGpu32fC1::Ptr getDisparities() override {return u_;}
  virtual inline ImageGpu32fC1::Ptr getOcclusion() override {return occ_;}

protected:
  ImageGpu32fC1::Ptr u_; //!< disparities (result)
  std::unique_ptr<ImageGpu32fC1> u_prev_; //!< disparities results from previous iteration
  std::shared_ptr<ImageGpu32fC1> u0_; //!< disparities results from previous warp
  std::unique_ptr<ImageGpu32fC2> pu_; //!< dual variable for primal variable
  std::unique_ptr<ImageGpu32fC1> q_; //!< dual variable for data term
  std::unique_ptr<ImageGpu32fC1> ix_; //!< spatial gradients on moving (warped) image
  std::unique_ptr<ImageGpu32fC1> it_; //!< temporal gradients between warped and fixed image
  std::unique_ptr<ImageGpu32fC1> xi_; //!< preconditioner
  std::unique_ptr<ImageGpu32fC1> g_; //!< (edge) image for weighting the regularizer
  imp::cu::ImageGpu32fC1::Ptr occ_; //!< estimation of occluded pixels

  // textures
  std::shared_ptr<Texture2D> lambda_tex_; //!< For pointwise lambda
  std::shared_ptr<Texture2D> i1_tex_;
  std::shared_ptr<Texture2D> i2_tex_;
  std::shared_ptr<Texture2D> u_tex_;
  std::shared_ptr<Texture2D> u_prev_tex_;
  std::shared_ptr<Texture2D> u0_tex_;
  std::shared_ptr<Texture2D> pu_tex_;
  std::shared_ptr<Texture2D> q_tex_;
  std::shared_ptr<Texture2D> ix_tex_;
  std::shared_ptr<Texture2D> it_tex_;
  std::shared_ptr<Texture2D> xi_tex_;
  std::shared_ptr<Texture2D> g_tex_;
  std::shared_ptr<Texture2D> occ_tex_;

};

} // namespace cu
} // namespace imp

#endif // IMP_CU_STEREO_CTF_WARPING_LEVEL_PRECOND_HUBER_L1_WEIGHTED_CUH
