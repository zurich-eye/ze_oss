#include <cmath>

#include <ze/common/test/entrypoint.h>
#include <ze/common/manifold.h>

template<typename T>
void testManifoldInvariants(const T& a, const T& b, double tol = 1e-9)
{
  EXPECT_TRUE(ze::traits<T>::Equals(a, a));
  typename ze::traits<T>::TangentVector v = ze::traits<T>::Local(a, b);
  T c = ze::traits<T>::Retract(a, v);
  EXPECT_TRUE(ze::traits<T>::Equals(b, c, tol));
}


TEST(ManifoldTests, testScalarTraits)
{
  EXPECT_EQ(ze::traits<double>::dimension, 1);
  testManifoldInvariants<double>(1.0, 1.5);
}

TEST(ManifoldTests, testEigenTraits)
{

  EXPECT_EQ(ze::traits<Eigen::Vector3d>::dimension, 3);
  testManifoldInvariants<Eigen::Vector3d>(Eigen::Vector3d(1.0, 1.2, 1.3),
                                          Eigen::Vector3d(2.0, 1.0, 0.0));
}

ZE_UNITTEST_ENTRYPOINT
