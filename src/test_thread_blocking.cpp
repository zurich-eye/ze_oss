#include <ze/common/test_thread_blocking.hpp>
#include <thread>
#include <chrono>
#include <ze/common/logging.hpp>

namespace ze {

//------------------------------------------------------------------------------
void BlockingTest::runBlockingTest(unsigned testId, unsigned timeout)
{
  test_id_ = testId;
  std::thread thread(&BlockingTest::runThread, this);

  std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
  CHECK(!finished_) << "Thread was not blocked";
  performUnblockingAction(testId);

  std::this_thread::sleep_for(std::chrono::milliseconds(timeout));
  CHECK(finished_) << "Thread was not unblocked";
  thread.join();
}

//------------------------------------------------------------------------------
void BlockingTest::runThread()
{
  performBlockingAction(testId());
  setFinished();
}

} // namespace ze
