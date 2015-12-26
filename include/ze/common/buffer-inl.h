#pragma once

namespace ze {

template <typename Scalar, int Dim>
bool Buffer<Scalar,Dim>::getNearestValue(int64_t stamp, Vector* value)
{
  CHECK_NOTNULL(value);
  CHECK_GE(stamp, 0);

  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
  {
    LOG(WARNING) << "Buffer is empty.";
    return false;
  }

  auto it_before = iterator_equal_or_before(stamp);
  if(it_before->first == stamp)
  {
    *value = it_before->second;
    return true;
  }

  // Compute time difference between stamp and closest entries.
  auto it_after = iterator_equal_or_after(stamp);
  int64_t dt_after = -1, dt_before = -1;
  if(it_after != buffer_.end())
    dt_after = it_after->first - stamp;
  if(it_before != buffer_.end())
    dt_before = stamp - it_before->first;

  // Select which entry is closest based on time difference.
  if(dt_after < 0 && dt_before < 0)
  {
    CHECK(false) << "Should not occur.";
    return false;
  }
  else if(dt_after < 0)
    *value = it_before->second;
  else if(dt_before < 0)
    *value = it_after->second;
  else if(dt_after > 0 && dt_before > 0 && dt_after < dt_before)
    *value = it_after->second;
  else
    *value = it_before->second;
  return true;
}

template <typename Scalar, int Dim>
bool Buffer<Scalar,Dim>::getOldestValue(Vector* value) const
{
  CHECK_NOTNULL(value);
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
    return false;
  *value = buffer_.begin()->second;
  return true;
}

template <typename Scalar, int Dim>
bool Buffer<Scalar,Dim>::getNewestValue(Vector* value) const
{
  CHECK_NOTNULL(value);
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.empty())
    return false;
  *value = buffer_.rbegin()->second;
  return true;
}

template <typename Scalar, int Dim>
typename Buffer<Scalar,Dim>::VectorBuffer::iterator Buffer<Scalar,Dim>::iterator_equal_or_before(int64_t stamp)
{
  CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  auto it = buffer_.lower_bound(stamp);

  if(it->first == stamp)
    return it; // Return iterator to key if exact key exists.
  if(stamp > buffer_.rbegin()->first)
    return (--buffer_.end()); // Pointer to last value.
  if(it == buffer_.begin())
    return buffer_.end(); // Invalid if data before first value.
  --it;
  return it;
}

template <typename Scalar, int Dim>
typename Buffer<Scalar,Dim>::VectorBuffer::iterator Buffer<Scalar,Dim>::iterator_equal_or_after(int64_t stamp)
{
  CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
  return buffer_.lower_bound(stamp);
}

} // namespace ze
