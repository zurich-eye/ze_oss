#include <ze/common/test_entrypoint.h>
#include <ze/common/sampler.h>

TEST(SamplerTest, testSampler)
{
  using namespace ze;

  Matrix2 Sigma;
  Sigma << 2, 0, 0, 3;
  GaussianSampler<2> sampler(Sigma);
  GaussianSampler<3> sampler2(Vector3(1, 2, 3));

  Vector2 sample = sampler.sample();
  Vector3 sample2 = sampler2.sample();
}

ZE_UNITTEST_ENTRYPOINT
