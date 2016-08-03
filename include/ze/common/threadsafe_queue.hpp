/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2013
 *      Author: Simon Lynen
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *    Modified: cfo / ZE
 *               - ZE Style
 *               - ported everything to c++11
 *               - removed polymorphism
 *********************************************************************************/

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>

#include <ze/common/logging.hpp>

namespace ze {

/**
 * @brief Class that implements a threadsafe FIFO queue.
 * @tparam QueueType Datatype that is safed in the queue.
 */
template<typename QueueType>
class ThreadSafeQueue
{
public:
  ThreadSafeQueue() = default;

  //! Notify all waiting threads. Only used in destructor and when shutting down.
  void notifyAll() const
  {
    condition_empty_.notify_all();
    condition_full_.notify_all();
  }

  ~ThreadSafeQueue()
  {
    shutdown_ = true;
    notifyAll();
  }

  //! Tell the queue shut down. This will notify all threads to wake up.
  void shutdown()
  {
    shutdown_ = true;
    notifyAll();
  }

  //! Tell the queue to resume after a shutdown request.
  void resume()
  {
    shutdown_ = false;
    notifyAll();
  }

  //! Push non-blocking to the queue.
  void push(const QueueType& value)
  {
    std::lock_guard lock(mutex_);
    queue_.push(value);
    condition_empty_.notify_one();
  }

  //! Return the size of the queue.
  size_t size() const
  {
    std::lock_guard lock(mutex_);
    return queue_.size();
  }

  //! Return true if the queue is empty.
  bool empty() const
  {
    std::lock_guard lock(mutex_);
    return queue_.empty();
  }

  //! Push to the queue if the size is less than max_queue_size, else block.
  //! \param[in] value New entry in queue.
  //! \param[in] max_queue_size Maximum queue size.
  //! \return False if shutdown is requested.
  bool pushBlockingIfFull(const QueueType& value, size_t max_queue_size)
  {
    std::lock_guard lock(mutex_);

    condition_full_.wait(lock,
                         [&]() { return queue_.size() < max_queue_size || shutdown_; });
    if (queue_.size() >= max_queue_size)
    {
      CHECK(shutdown_);
      return false;
    }

    queue_.push(value);
    condition_empty_.notify_all();  // Signal that data is available.
    return true;
  }

  //! Push to the queue. If full, drop the oldest entry.
  //! \param[in] value New entry in queue.
  //! \param[in] max_queue_size Maximum queue size.
  //! \return True if oldest was dropped because queue was full.
  bool pushNonBlockingDroppingIfFull(
      const QueueType& value,
      size_t max_queue_size)
  {
    std::lock_guard lock(mutex_);
    bool result = false;
    if (queue_.size() >= max_queue_size)
    {
      queue_.pop();
      result = true;
    }
    queue_.push(value);
    condition_empty_.notify_all();  // Signal that data is available.
    return result;
  }

  //! Get the oldest entry still in the queue. Blocking if queue is empty.
  //! @param[out] value Oldest entry in queue.
  //! @return False if shutdown is requested.
  bool popBlocking(QueueType* value)
  {
    CHECK_NOTNULL(value);
    std::lock_guard lock(mutex_);

    condition_empty_.wait(lock, [&](){ return !queue_.empty() || shutdown_; });

    if (queue_.empty())
    {
      CHECK(shutdown_);
      return false;
    }

    *value = queue_.front();
    queue_.pop();
    condition_full_.notify_all();  // Notify that space is available.
    return true;
  }

  //! Get the oldest entry still in the queue. If queue is empty value is not altered.
  //! @param[out] value Oldest entry in queue if queue was not empty.
  //! @return True if queue was not empty.
  bool popNonBlocking(QueueType* value)
  {
    CHECK_NOTNULL(value);
    std::lock_guard lock(mutex_);
    if (queue_.empty())
    {
      return false;
    }
    *value = queue_.front();
    queue_.pop();
    condition_full_.notify_all();  // Notify that space is available.
    return true;
  }

  //! @brief Get the oldest entry still in the queue. If the queue is empty wait for a given
  //!        amount of time. If during this time an entry was pushed alter the value. If the
  //!        queue is still empty, the value is not altered and it will return false
  //! @param[out] value Oldest entry in queue if queue was not empty.
  //! @param  timeout_nanoseconds Maximum amount of time to wait for an entry if queue is empty.
  //! @return True if value was updated. False if queue was empty and no new entry was pushed
  //!         during the given timeout.
  bool popTimeout(QueueType* value, int64_t timeout_nanoseconds)
  {
    CHECK_NOTNULL(value);
    std::lock_guard lock(mutex_);

    auto wait_until = std::chrono::system_clock::now()
                      + std::chrono::nanoseconds(timeout_nanoseconds);
    condition_empty_.wait_until(lock,
                                wait_until,
                                [&](){ return !queue_.empty() || shutdown_; });

    if (queue_.empty())
    {
      return false;
    }

    *value = queue_.front();
    queue_.pop();
    condition_full_.notify_all(); // Notify that space is available.
    return true;
  }

private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_empty_;  //!< Signal that queue is not empty.
  mutable std::condition_variable condition_full_;   //!< Signal when an element is popped.
  std::queue<QueueType> queue_;
  std::atomic_bool shutdown_ { false };              //!< Flag if shutdown is requested.
};

}  // namespace ze
