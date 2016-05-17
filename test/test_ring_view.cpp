#include <cmath>

#include <ze/common/types.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/ring_view.h>

TEST(RingViewTest, testFullRingScalar)
{
  using namespace ze;

  std::vector<int> vec(100);
  ring_view<int> rv(vec.begin(), vec.end());
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(100, rv.size());
  ASSERT_EQ(99, rv.back_idx());
  rv.push_back(1);
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(100, rv.size());
  ASSERT_EQ(0, rv.back_idx());
  rv.pop_front();
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(99, rv.size());
  ASSERT_EQ(0, rv.back_idx());
}

TEST(RingViewTest, testFullRingScalarFixedSIze)
{
  using namespace ze;

  std::vector<int> vec(100);
  ring_view<int, 100> rv(vec.begin(), vec.end());
  EXPECT_EQ(100, rv.capacity());
}


TEST(RingViewTest, testEmptyRingScalar)
{
  using namespace ze;

  std::vector<int> vec(100);
  ring_view<int> rv(vec.begin(), vec.end(), vec.begin(), 0);
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(0, rv.size());
  rv.push_back(1);
  ASSERT_EQ(1, rv.at(0));
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(1, rv.size());
  rv.pop_front();
  ASSERT_EQ(100, rv.capacity());
  ASSERT_EQ(0, rv.size());
}

TEST(RingViewTest, testRingEigenVector)
{
  using namespace ze;

  Eigen::Matrix<FloatType, 1, 4> data;
  data << 1, 2, 3, 4;

  ring_view<FloatType> rv(data.data(), data.data()+4);
  ASSERT_EQ(1, rv.at(0));
  ASSERT_EQ(2, rv.at(1));
  ASSERT_EQ(3, rv.at(2));
  ASSERT_EQ(4, rv.at(3));

  rv.push_back(7);
  ASSERT_EQ(2, rv.at(0));
  ASSERT_EQ(7, rv.at(3));
}

TEST(RingViewTest, testIteratorOperators)
{
  using namespace ze;
  Eigen::Matrix<FloatType, 1, 4> data;
  data << 1, 2, 3, 4;

  ring_view<FloatType> rv(data.data(), data.data()+4);

  auto iter1 = rv.begin();
  auto iter2 = rv.end();

  EXPECT_TRUE(iter1 < iter2);
  EXPECT_TRUE(iter1 <= iter2);
  EXPECT_TRUE(iter2 > iter1);
  EXPECT_TRUE(iter2 >= iter1);
  EXPECT_FALSE(iter1 == iter2);
  EXPECT_TRUE(iter1 == iter1);
  EXPECT_TRUE(iter1 != iter2);

  iter1 += 2;
  iter2 -= 2;

  EXPECT_TRUE(iter1 == iter2);
}

TEST(RingViewTest, testResetFront)
{
  using namespace ze;

  std::vector<int> vec = {1, 2, 3, 4};
  ring_view<int> rv(vec.begin(), vec.end());

  EXPECT_EQ(4, rv.size());
  EXPECT_EQ(0, rv.begin().index());
  EXPECT_EQ(4, rv.end().index());

  rv.reset_front(2);
  EXPECT_EQ(2, rv.size());
  EXPECT_EQ(3, rv.at(0));

  rv.reset_front(0);
  EXPECT_EQ(4, rv.size());
  EXPECT_EQ(1, rv.at(0));
}

ZE_UNITTEST_ENTRYPOINT

