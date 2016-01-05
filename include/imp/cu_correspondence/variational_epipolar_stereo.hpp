#ifndef IMP_CU_VARIATIONAL_EPIPOLAR_STEREO_HPP
#define IMP_CU_VARIATIONAL_EPIPOLAR_STEREO_HPP


#include <cstdint>
#include <memory>

#include <imp/cu_core/cu_matrix.cuh>
#include <imp/cu_core/cu_se3.cuh>
#include <imp/cu_core/cu_pinhole_camera.cuh>
#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/cu_correspondence/variational_stereo_parameters.hpp>
#include <imp/cu_correspondence/variational_stereo.hpp>

namespace imp {
namespace cu {

/**
 * @brief The Stereo class takes an image pair with known epipolar geometry
 *        (fundamental matrix) and estimates the disparity map
 */
class VariationalEpipolarStereo : public VariationalStereo
{
public:
  using VectorImage = imp::cu::ImageGpu32fC2;
  using Cameras = std::vector<cu::PinholeCamera>;

public:
  VariationalEpipolarStereo(Parameters::Ptr params=nullptr);
  virtual ~VariationalEpipolarStereo(); //= default;

  virtual void setFundamentalMatrix(const cu::Matrix3f& F);
  virtual void setIntrinsics(const Cameras& cams);
  virtual void setExtrinsics(const cu::SE3<float>& T_mov_fix);
  virtual void setDepthProposal(
      const ImageGpu32fC1::Ptr& depth_proposal,
      const ImageGpu32fC1::Ptr& depth_proposal_sigma2=nullptr);

private:
};

} // namespace cu
} // namespace imp

#endif // IMP_CU_STEREO_HPP
