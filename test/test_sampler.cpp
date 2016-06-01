#include <ze/common/test_entrypoint.h>
#include <ze/common/sampler.h>

TEST(SamplerTest, testSampler)
{
  using namespace ze;

  Eigen::Matrix2d Sigma;
  Sigma << 2, 0, 0, 3;
  GaussianSampler<2> sampler(Sigma);
  GaussianSampler<3> sampler2(Vector3(1, 2, 3));

  Eigen::Vector2d sample = sampler.sample();
  Eigen::Vector3d sample2 = sampler2.sample();
}

ZE_UNITTEST_ENTRYPOINT
