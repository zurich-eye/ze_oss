#pragma once

#include <map>
#include <thread>
#include <mutex>
#include <Eigen/Dense>
#include <glog/logging.h>

#include <ze/common/types.h>
#include <ze/common/time.h>

namespace ze {

// Oldest timestamp: buffer.begin(), Newest Timestamp: buffer.end() / buffer.rbegin()
template <typename Scalar, int Dim>
class Buffer
{
public:
  using Vector = Eigen::Matrix<Scalar, Dim, 1>;
  using VectorBuffer = std::map<int64_t, Vector, std::less<int64_t>, Eigen::aligned_allocator<Vector>>;

  static constexpr int kDim = Dim;

  inline void insert(int64_t stamp, const Vector& data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_[stamp] = data;
  }

  // Get value with timestamp closest to stamp. Returns false if buffer empty.
  bool getNearestValue(int64_t stamp, Vector* value);

  bool getOldestValue(Vector* value) const;

  bool getNewestValue(Vector* value) const;

  bool getBetweenValuesInterpolated(
      int64_t stamp_from, int64_t stamp_to,
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* stamps,
      Eigen::Matrix<Scalar, kDim, Eigen::Dynamic>* values);

  inline void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.clear();
  }

  inline void size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.size();
  }

  inline void empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    buffer_.empty();
  }

  inline void removeDataBeforeTimestamp(int64_t stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    removeDataBeforeTimestamp_impl(stamp);
  }

  inline void removeDataOlderThan(double seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(buffer_.empty())
      return;

    removeDataBeforeTimestamp_impl(
          buffer_.rbegin()->first - time::secToNanosec(seconds));
  }

  inline void lock() const
  {
    mutex_.lock();
  }

  inline void unlock() const
  {
    mutex_.unlock();
  }

  const VectorBuffer& data() const
  {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return buffer_;
  }

  typename VectorBuffer::iterator iterator_equal_or_before(int64_t stamp);

  typename VectorBuffer::iterator iterator_equal_or_after(int64_t stamp);

protected:
  mutable std::mutex mutex_;
  VectorBuffer buffer_;

  inline void removeDataBeforeTimestamp_impl(int64_t stamp)
  {
    auto it = buffer_.lower_bound(stamp);
    buffer_.erase(buffer_.begin(), it);
  }
};

} // namespace ze

#include <ze/common/buffer-inl.h>
