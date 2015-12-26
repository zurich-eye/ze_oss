#include <string>
#include <vector>

#include <ze/common/buffer.h>
#include <ze/common/test/entrypoint.h>


TEST(BufferTest, testRemoveOlderThanTimestamp)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
    buffer.insert(i, Eigen::Vector2d(i, i));

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
    buffer.insert(ze::time::secToNanosec(i), Eigen::Vector2d(i, i));

  buffer.removeDataOlderThan(3.0);
  buffer.lock();
  EXPECT_EQ(buffer.data().begin()->first, ze::time::secToNanosec(6));
  EXPECT_EQ(buffer.data().rbegin()->first, ze::time::secToNanosec(9));
  buffer.unlock();
}

TEST(BufferTest, testIterator)
{
  ze::Buffer<double, 2> buffer;
  for(int i = 1; i < 10; ++i)
    buffer.insert(ze::time::secToNanosec(i), Eigen::Vector2d(i, i));

  buffer.lock();

  // Check before/after
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::time::secToNanosec(3.5))->first,
            ze::time::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::time::secToNanosec(3.5))->first,
            ze::time::secToNanosec(4));

  // Check equal
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::time::secToNanosec(3))->first,
            ze::time::secToNanosec(3));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::time::secToNanosec(4))->first,
            ze::time::secToNanosec(4));

  // Expect out of range:
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::time::secToNanosec(0.8)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_before(ze::time::secToNanosec(9.1)),
            (--buffer.data().end()));
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::time::secToNanosec(9.1)),
            buffer.data().end());
  EXPECT_EQ(buffer.iterator_equal_or_after(ze::time::secToNanosec(0.8)),
            buffer.data().begin());

  buffer.unlock();
}

TEST(BufferTest, testNearestValue)
{
  Eigen::Vector2d value;
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(buffer.getNearestValue(ze::time::secToNanosec(1), &value));

  for(int i = 1; i < 10; ++i)
    buffer.insert(ze::time::secToNanosec(i), Eigen::Vector2d(i, i));

  buffer.getNearestValue(ze::time::secToNanosec(1), &value);
  EXPECT_EQ(value[0], 1);
  buffer.getNearestValue(ze::time::secToNanosec(0.4), &value);
  EXPECT_EQ(value[0], 1);
  buffer.getNearestValue(ze::time::secToNanosec(1.4), &value);
  EXPECT_EQ(value[0], 1);
  buffer.getNearestValue(ze::time::secToNanosec(11.0), &value);
  EXPECT_EQ(value[0], 9);
}

TEST(BufferTest, testOldestNewestValue)
{
  Eigen::Vector2d value;
  ze::Buffer<double, 2> buffer;
  EXPECT_FALSE(buffer.getOldestValue(&value));
  EXPECT_FALSE(buffer.getNewestValue(&value));

  for(int i = 1; i < 10; ++i)
    buffer.insert(ze::time::secToNanosec(i), Eigen::Vector2d(i, i));

  buffer.getNewestValue(&value);
  EXPECT_EQ(value[0], 9);
  buffer.getOldestValue(&value);
  EXPECT_EQ(value[0], 1);
}

ZE_UNITTEST_ENTRYPOINT
