#include <string>
#include <vector>

#include <ze/common/buffer.h>
#include <ze/common/test/entrypoint.h>


TEST(BufferTest, testRemoveOlderThanTimestamp)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(i, Eigen::Vector2d(i, i));
  }

  buffer.removeDataBeforeTimestamp(3);
  buffer.lock();
  EXPECT_EQ(buffer.data().begin()->first, 3);
  EXPECT_EQ(buffer.data().rbegin()->first, 9);
  buffer.unlock();
}

TEST(BufferTest, testRemoveOlderThan)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  buffer.removeDataOlderThan(3.0);
  buffer.lock();
  EXPECT_EQ(buffer.data().begin()->first, ze::secToNanosec(6));
  EXPECT_EQ(buffer.data().rbegin()->first, ze::secToNanosec(9));
  buffer.unlock();
}

TEST(BufferTest, testIterator)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  buffer.lock();

  // Check before/after
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(3.5))->first,
            ze::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(3.5))->first,
            ze::secToNanosec(4));

  // Check equal
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(3))->first,
            ze::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(4))->first,
            ze::secToNanosec(4));

  // Expect out of range:
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(0.8)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::secToNanosec(9.1)),
            (--buffer.data().end()));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(9.1)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::secToNanosec(0.8)),
            buffer.data().begin());

  buffer.unlock();
}

TEST(BufferTest, testNearestValue)
{
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(buffer.getNearestValue(ze::secToNanosec(1)).second);

  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  EXPECT_EQ(buffer.getNearestValue(ze::secToNanosec(1)).first[0], 1);
  EXPECT_EQ(buffer.getNearestValue(ze::secToNanosec(0.4)).first[0], 1);
  EXPECT_EQ(buffer.getNearestValue(ze::secToNanosec(1.4)).first[0], 1);
  EXPECT_EQ(buffer.getNearestValue(ze::secToNanosec(11.0)).first[0], 9);
}

TEST(BufferTest, testOldestNewestValue)
{
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(buffer.getOldestValue().second);
  EXPECT_FALSE(buffer.getNewestValue().second);

  for(int i = 1; i < 10; ++i)
  {
    buffer.insert(ze::secToNanosec(i), Eigen::Vector2d(i, i));
  }

  EXPECT_EQ(buffer.getNewestValue().first[0], 9);
  EXPECT_EQ(buffer.getOldestValue().first[0], 1);
}

TEST(BufferTest, testInterpolation)
{
  using namespace ze;

  Buffer<FloatType, 2> buffer;

  for(int i = 0; i < 10; ++i)
    buffer.insert(secToNanosec(i), Vector2(i, i));

  VectorX stamps;
  Eigen::Matrix<FloatType, 2, Eigen::Dynamic> values;
  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(
        secToNanosec(1.2), secToNanosec(5.4));
  EXPECT_EQ(stamps(0), secToNanosec(1.2));
  EXPECT_EQ(stamps(stamps.cols()-1), secToNanosec(5.4));
  EXPECT_DOUBLE_EQ(values(0, 0), 1.2);
  EXPECT_DOUBLE_EQ(values(0, stamps.cols()-1), 5.4);

  std::tie(stamps, values) = buffer.getBetweenValuesInterpolated(
        secToNanosec(0), secToNanosec(9));
  EXPECT_EQ(stamps(0), secToNanosec(0));
  EXPECT_EQ(stamps(stamps.cols()-1), secToNanosec(9));
  EXPECT_DOUBLE_EQ(values(0, 0), 0);
  EXPECT_DOUBLE_EQ(values(0, stamps.cols()-1), 9);
}

ZE_UNITTEST_ENTRYPOINT
