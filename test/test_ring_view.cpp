#include <cmath>

#include <ze/common/types.h>
#include <ze/common/test_entrypoint.h>
#include <ze/common/benchmark.h>
#include <ze/common/ring_view.h>

DEFINE_bool(run_benchmark, false, "Benchmark the buffer vs. ringbuffer");

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

  Eigen::Matrix<real_t, 1, 4> data;
  data << 1, 2, 3, 4;

  ring_view<real_t> rv(data.data(), data.data()+4);
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
  Eigen::Matrix<real_t, 1, 4> data;
  data << 1, 2, 3, 4;

  ring_view<real_t> rv(data.data(), data.data()+4);

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

TEST(RingViewTest, benchmarkFixedVsDynamicSize)
{
  if (!FLAGS_run_benchmark) {
    return;
  }

  using namespace ze;

  std::vector<int> vec(100);
  ring_view<int> rv1(vec.begin(), vec.end());
  ring_view<int, 100> rv2(vec.begin(), vec.end());

  std::vector<int> vec2(128);
  ring_view<int> rv3(vec2.begin(), vec2.end());
  ring_view<int, 128> rv4(vec2.begin(), vec2.end());

  //////
  // access
  auto atFixed = [&]() { rv2.at(26); };
  real_t atFixed_r = runTimingBenchmark(atFixed, 1000, 20, "At Fixed", true);
  auto atDynamic = [&]() { rv1.at(26); };
  real_t atDynamic_r = runTimingBenchmark(atDynamic, 1000, 20, "At Fixed", true);
  auto atFixed_128 = [&]() { rv4.at(26); };
  real_t atFixed128_r = runTimingBenchmark(atFixed_128, 1000, 20, "At Fixed", true);
  auto atDynamic_128 = [&]() { rv3.at(26); };
  real_t atDynamic128_r = runTimingBenchmark(atDynamic_128, 1000, 20, "At Fixed", true);

  VLOG(1) << "Fixed: " << atFixed_r << ", Fixed128: " << atFixed128_r << "\n";
  VLOG(1) << "Dynamic: " << atDynamic_r << ", Dynamic128: " << atDynamic128_r << "\n";

}
ZE_UNITTEST_ENTRYPOINT

