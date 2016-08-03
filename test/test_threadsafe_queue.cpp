#include <thread>

#include <ze/common/logging.hpp>
#include <ze/common/test_entrypoint.h>
#include <ze/common/threadsafe_queue.hpp>
#include <ze/common/time_conversions.h>

using namespace ze;

TEST(ThreadsafeQueueTest, testConstructor)
{
  ThreadSafeQueue<int> queue;
  for (int i = 0; i < 10; ++i)
  {
    queue.push(i);
  }

  // Check that values can be retrieved in same order.
  for (int i = 0; i < 10; ++i)
  {
    int data;
    bool res;
    std::tie(data, res) = queue.popBlocking();
    EXPECT_TRUE(res);
    EXPECT_EQ(data, i);
  }
  queue.shutdown();
}

TEST(ThreadsafeQueueTest, testMultithreadedDynamicSize)
{
  ThreadSafeQueue<int> queue;

  std::thread thread1([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      queue.push(i);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  });

  std::thread thread2([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      int data;
      bool res;
      std::tie(data, res) = queue.popBlocking();
      ASSERT_TRUE(res);
      EXPECT_EQ(data, i);
    }
  });

  thread1.join();
  thread2.join();
  queue.shutdown();
}

TEST(ThreadsafeQueueTest, testMultithreadedFixedSizeBlocking)
{
  ThreadSafeQueue<int> queue;
  constexpr size_t queue_size = 10;

  std::thread thread1([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      queue.pushBlockingIfFull(i, queue_size);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  });

  std::thread thread2([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      int data;
      bool res;
      std::tie(data, res) = queue.popBlocking();
      ASSERT_TRUE(res);
      EXPECT_EQ(data, i);
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
  });

  thread1.join();
  thread2.join();
  queue.shutdown();
}

TEST(ThreadsafeQueueTest, testMultithreadedFixedSizeNonBlocking)
{
  ThreadSafeQueue<int> queue;
  constexpr size_t queue_size = 10;

  std::thread thread1([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      queue.pushBlockingIfFull(i, queue_size);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  });

  std::thread thread2([&]()
  {
    int j = 0;
    for (int i = 0; i < 1000; ++i)
    {
      int data;
      bool res;
      std::tie(data, res) = queue.popNonBlocking();
      if (res)
      {
        EXPECT_EQ(data, j);
        ++j;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(50));
    }
  });

  thread1.join();
  thread2.join();
  queue.shutdown();
}

TEST(ThreadsafeQueueTest, testMultithreadedFixedSizeNonBlockingPushDropping)
{
  ThreadSafeQueue<int> queue;
  constexpr size_t queue_size = 10;

  std::thread thread1([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      queue.pushNonBlockingDroppingIfFull(i, queue_size);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  });

  std::thread thread2([&]()
  {
    int j = 0;
    for (int i = 0; i < 1000; ++i)
    {
      int data;
      bool res;
      std::tie(data, res) = queue.popNonBlocking();
      if (res)
      {
        EXPECT_GE(data, j);
        j = data;
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  thread1.join();
  thread2.join();
  queue.shutdown();
}

TEST(ThreadsafeQueueTest, testMultithreadedDynamicSizeWaiting)
{
  ThreadSafeQueue<int> queue;

  std::thread thread1([&]()
  {
    for (int i = 0; i < 1000; ++i)
    {
      queue.push(i);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(50));
  });

  std::thread thread2([&]()
  {
    int j = 0;
    for (int i = 0; i < 1000; ++i)
    {
      int data;
      bool res;
      std::tie(data, res) = queue.popTimeout(millisecToNanosec(0.1));
      if (res)
      {
        EXPECT_GE(data, j);
        j = data;
      }
    }
  });

  thread1.join();
  thread2.join();
  queue.shutdown();
}




ZE_UNITTEST_ENTRYPOINT
