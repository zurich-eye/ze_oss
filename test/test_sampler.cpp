#include <ze/common/test_entrypoint.h>
#include <ze/common/sampler.hpp>

TEST(SamplerTest, testSampler)
{
  using namespace ze;

  Vector2 var_vector;
  var_vector << 2, 3;
  GaussianSampler<2>::Ptr sampler = GaussianSampler<2>::variances(var_vector);
  GaussianSampler<3>::Ptr sampler2 = GaussianSampler<3>::sigmas(Vector3(1, 2, 3));

  Vector2 sample = sampler->sample();
  Vector3 sample2 = sampler2->sample();
}

ZE_UNITTEST_ENTRYPOINT
