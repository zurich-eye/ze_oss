#ifndef IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH
#define IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH

#include <cstdint>
#include <memory>

#include <imp/cu_correspondence/solver_stereo_abstract.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/core/size.hpp>

namespace imp {
namespace cu {

// forward decl
class VariationalStereoParameters;
class Texture2D;

/**
 * @brief The StereoCtFWarpingLevelPrecondHuberL1 class
 */
class SolverEpipolarStereoPrecondHuberL1 : public SolverStereoAbstract
{
public:
  SolverEpipolarStereoPrecondHuberL1() = delete;
  virtual ~SolverEpipolarStereoPrecondHuberL1();

  SolverEpipolarStereoPrecondHuberL1(const Parameters::Ptr& params,
                                     imp::Size2u size, size_t level,
                                     const std::vector<cu::PinholeCamera>& cams,
                                     const cu::Matrix3f& F,
                                     const cu::SE3<float>& T_mov_fix,
                                     const ImageGpu32fC1& depth_proposal,
                                     const imp::cu::ImageGpu32fC1& depth_proposal_sigma2);

  virtual void init();
  virtual void init(const SolverStereoAbstract& rhs);
  virtual inline void setFundamentalMatrix(const cu::Matrix3f& F) {F_ = F;}

  virtual void solve(std::vector<ImageGpu32fC1::Ptr> images);

  virtual inline ImageGpu32fC1::Ptr getDisparities() {return u_;}

protected:
  ImageGpu32fC1::Ptr u_; //!< disparities (result)
  std::unique_ptr<ImageGpu32fC1> u_prev_; //!< disparities results from previous iteration
  std::unique_ptr<ImageGpu32fC1> u0_; //!< disparities results from previous warp
  std::unique_ptr<ImageGpu32fC2> pu_; //!< dual variable for primal variable
  std::unique_ptr<ImageGpu32fC1> q_; //!< dual variable for data term
  std::unique_ptr<ImageGpu32fC1> iw_; //!< warped moving image
  std::unique_ptr<ImageGpu32fC1> ix_; //!< spatial gradients on moving (warped) image
  std::unique_ptr<ImageGpu32fC1> it_; //!< temporal gradients between warped and fixed image
  std::unique_ptr<ImageGpu32fC1> xi_; //!< preconditioner
  std::unique_ptr<ImageGpu32fC1> g_; //!< for edge weighting

  cu::Matrix3f F_;
  std::vector<cu::PinholeCamera> cams_;
  cu::SE3<float> T_mov_fix_;
  std::unique_ptr<ImageGpu32fC1> depth_proposal_;
  std::unique_ptr<ImageGpu32fC1> depth_proposal_sigma2_;

  // textures
  std::shared_ptr<Texture2D> lambda_tex_;
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

  std::shared_ptr<Texture2D> depth_proposal_tex_;
  std::shared_ptr<Texture2D> depth_proposal_sigma2_tex_;

//  std::shared_ptr<Texture2D> correspondence_guess_tex_;
//  std::shared_ptr<Texture2D> epi_vec_tex_;

};

} // namespace cu
} // namespace imp

#endif // IMP_CU_EPIPOLAR_STEREO_PRECOND_HUBER_L1_CUH
