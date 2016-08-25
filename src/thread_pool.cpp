// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/common/thread_pool.hpp>

namespace ze {

void ThreadPool::startThreads(size_t threads)
{
  for(size_t i = 0;i<threads;++i)
  {
    workers_.emplace_back(
      [this] {

        // Thread loop:
        while (true)
        {
          std::function<void()> task;

          // Wait for next task.
          {
            std::unique_lock<std::mutex> lock(this->queue_mutex_);
            this->condition_.wait(lock, [this]{ return this->stop_ || !this->tasks_.empty(); });
            if (this->stop_ && this->tasks_.empty())
            {
              return;
            }
            task = std::move(this->tasks_.front());
            this->tasks_.pop();
          }

          // Execute task.
          task();
        }
      }
    );
  }
}

ThreadPool::~ThreadPool()
{
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    stop_ = true;
  }
  condition_.notify_all();
  for (std::thread& worker : workers_)
  {
    worker.join();
  }
}

} // namespace ze
