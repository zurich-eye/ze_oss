#ifndef IMP_CU_STEREOCTFWARPING_HPP
#define IMP_CU_STEREOCTFWARPING_HPP

#include <memory>
#include <vector>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_imgproc/image_pyramid.hpp>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_matrix.cuh>

#include <imp/cu_correspondence/variational_stereo_parameters.hpp>
//#include <imp/cu_correspondence/solver_stereo_abstract.hpp>


namespace imp {
namespace cu {

// forward declarations
class SolverStereoAbstract;

/**
 * @brief The StereoCtFWarping class
 * @todo (MWE) better handling of fixed vs. moving images when adding (incremental updates)
 * @todo (MWE) better interface for multiple input images with fundamental matrix prior
 */
class StereoCtFWarping
{
public:
  using Parameters = VariationalStereoParameters;

  using ImageGpu32fC1 = imp::cu::ImageGpu32fC1;
  using ImageGpu32fC2 = imp::cu::ImageGpu32fC2;
  using ImagePyramid32fC1 = imp::ImagePyramid32fC1;

  using Cameras = std::vector<cu::PinholeCamera>;
  using CamerasPyramid = std::vector<Cameras>;

public:
  StereoCtFWarping() = delete;
  virtual ~StereoCtFWarping();// = default;
//  StereoCtFWarping(const StereoCtFWarping&);
//  StereoCtFWarping(StereoCtFWarping&&);

  StereoCtFWarping(Parameters::Ptr params);

  void addImage(const ImageGpu32fC1::Ptr& image);
  void solve();
  ImageGpu32fC1::Ptr computePrimalEnergy(size_t level=0);
  ImageGpu32fC1::Ptr getDisparities(size_t level=0);
  ImageGpu32fC1::Ptr getOcclusion(size_t level=0);

  // if we have a guess about the correspondence points and the epipolar geometry
  // given we can set these as a prior
  inline virtual void setFundamentalMatrix(const cu::Matrix3f& F) {F_ = F;}
  virtual void setIntrinsics(const Cameras& cams) {cams_ = cams;}
  virtual void setExtrinsics(const cu::SE3<float>& T_mov_fix) {T_mov_fix_=T_mov_fix;}

  inline virtual void setDepthProposal(
      const ImageGpu32fC1::Ptr& depth_proposal,
      const ImageGpu32fC1::Ptr& depth_proposal_sigma2=nullptr)
  {
    depth_proposal_ = depth_proposal;
    depth_proposal_sigma2_ = depth_proposal_sigma2;
  }

protected:
  /**
   * @brief ready checks if everything is setup and initialized.
   * @return State if everything is ready to solve the given problem.
   */
  bool ready();

  /**
   * @brief init initializes the solvers for the current setup
   */
  void init();

private:
  Parameters::Ptr params_; //!< configuration parameters
  std::vector<ImageGpu32fC1::Ptr> images_; //!< all unprocessed input images
  std::vector<ImagePyramid32fC1::Ptr> image_pyramids_; //!< image pyramids corresponding to the unprocesed input images
  std::vector<std::unique_ptr<SolverStereoAbstract>> levels_;

  cu::Matrix3f F_;
  std::vector<cu::PinholeCamera> cams_;
  cu::SE3<float> T_mov_fix_;

  ImageGpu32fC1::Ptr depth_proposal_;
  ImageGpu32fC1::Ptr depth_proposal_sigma2_;
};

} // namespace cu
} // namespace imp

#endif // IMP_CU_STEREOCTFWARPING_HPP
