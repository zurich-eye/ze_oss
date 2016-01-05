#ifndef VARIATIONAL_STEREO_PARAMETERS_HPP
#define VARIATIONAL_STEREO_PARAMETERS_HPP

#include <cstdint>
#include <imp/core/types.hpp>
#include <imp/cu_correspondence/stereo_solver_enum.hpp>
#include <imp/cu_core/cu_image_gpu.cuh>

namespace imp {
namespace cu {

// the parameter struct
struct VariationalStereoParameters
{
  using Ptr = typename std::shared_ptr<VariationalStereoParameters>;

  int verbose=10; //!< verbosity level (the higher, the more the Stereo algorithm talks to us)
  StereoPDSolver solver=StereoPDSolver::PrecondHuberL1; //!< selected primal-dual solver / model
  float lambda = 30.0f; //!< tradeoff between regularization and matching term
  ImageGpu32fC1::Ptr lambda_pointwise = nullptr; //!< pointwise variant of lambda
  float eps_u = 0.05f; //!< tradeoff between L1 and L2 part of the Huber regularization

  float edge_sigma = 1.f;
  float edge_alpha = 7.f;
  float edge_q = 0.7f;

  // settings for the ctf warping
  struct CTF // we might want to define this externally for all ctf approaches?
  {
    float scale_factor = 0.8f; //!< multiplicative scale factor between coarse-to-fine pyramid levels
    std::uint32_t iters = 100;
    std::uint32_t warps =  10;
    size_t levels = UINT32_MAX;
    size_t coarsest_level = UINT32_MAX;
    size_t finest_level = 0;
    bool apply_median_filter = true;
  } ctf;

  friend std::ostream& operator<<(std::ostream& stream,
                                  const VariationalStereoParameters& p);
};


} // namespace cu
} // namespace imp

#endif // VARIATIONAL_STEREO_PARAMETERS_HPP

