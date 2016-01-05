#ifndef IMP_CU_SOLVER_STEREO_ABSTRACT_HPP
#define IMP_CU_SOLVER_STEREO_ABSTRACT_HPP

#include <cstdint>
#include <memory>

#include <imp/cu_core/cu_image_gpu.cuh>
#include <imp/core/size.hpp>
#include <imp/cu_correspondence/variational_stereo_parameters.hpp>


namespace imp {
namespace cu {

/**
 * @brief The StereoCtFWarpingLevel class
 */
class SolverStereoAbstract
{
public:
  using ImageGpu32fC1 = imp::cu::ImageGpu32fC1;
  using ImageGpu32fC2 = imp::cu::ImageGpu32fC2;
  using Parameters = VariationalStereoParameters;

public:
  SolverStereoAbstract() = delete;
  virtual ~SolverStereoAbstract() = default;

  SolverStereoAbstract(Parameters::Ptr params,
                       imp::Size2u size, std::uint16_t level)
    : params_(params)
    , size_(size)
    , level_(level)
  { ; }

  virtual void init() = 0;
  virtual void init(const SolverStereoAbstract& rhs) = 0;
  virtual void solve(std::vector<ImageGpu32fC1::Ptr> images) = 0;

  /**
   * @brief computePrimalEnergy returns an the primal energy with the current disparity values.
   * @note There is no need to implement this function so by default a nullptr is returned
   * @return Pixel-wise primal energy
   */
  virtual ImageGpu32fC1::Ptr computePrimalEnergy() {return nullptr;}

  virtual ImageGpu32fC1::Ptr getDisparities() = 0;

  /**
   * @brief getOcclusion returns an estimate of occluded pixels
   * @note There is no need to implement this function so by default a nullptr is returned
   * @return A mask with an estimate of occluded pixels or nullptr if not estimated.
   */
  virtual ImageGpu32fC1::Ptr getOcclusion() {return nullptr;}


  // setters / getters
  inline imp::Size2u size() { return size_; }
  inline std::uint16_t level() { return level_; }

protected:
  Parameters::Ptr params_; //!< configuration parameters
  imp::Size2u size_;
  std::uint16_t level_; //!< level number in the ctf pyramid (0=finest .. n=coarsest)
};

} // namespace cu
} // namespace imp

#endif // IMP_CU_SOLVER_STEREO_ABSTRACT_HPP
