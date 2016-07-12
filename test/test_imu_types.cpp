#include <ze/common/test_entrypoint.h>

#include <ze/imu/imu_types.hpp>

TEST(ImuModelTest, testImuGetAccGetGyr)
{
  using namespace ze;

  ImuAccGyr m = ImuAccGyr::Random();

  // Mutable:
  getAcc(m) = Vector3(1, 2, 3);
  EXPECT_EQ(Vector3(1, 2, 3).eval(), m.head<3>());
  getGyr(m) = Vector3(4, 5, 6);
  EXPECT_EQ(Vector3(4, 5, 6).eval(), m.tail<3>());

  const ImuAccGyr m_const = m;
  EXPECT_EQ(getAcc(m), getAcc(m_const));
  EXPECT_EQ(getGyr(m), getGyr(m_const));
}

ZE_UNITTEST_ENTRYPOINT
