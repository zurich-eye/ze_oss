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
bool Buffer<Scalar,Dim>::getBetweenValuesInterpolated(
    int64_t stamp_from, int64_t stamp_to,
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic>* stamps,
    Eigen::Matrix<Scalar, kDim, Eigen::Dynamic>* values)
{
  CHECK_NOTNULL(stamps);
  CHECK_NOTNULL(values);
  CHECK_GE(stamp_from, 0);
  CHECK_LT(stamp_from, stamp_to);
  std::lock_guard<std::mutex> lock(mutex_);
  if(buffer_.size() < 2)
  {
    LOG(WARNING) << "Buffer has less than 2 entries.";
    return false;
  }

  const int64_t oldest_stamp = buffer_.begin()->first;
  const int64_t newest_stamp = buffer_.rbegin()->first;
  if(stamp_from < oldest_stamp)
  {
    LOG(WARNING) << "Requests older timestamp than in buffer.";
    return false;
  }
  if(stamp_to > newest_stamp)
  {
    LOG(WARNING) << "Requests newer timestamp than in buffer.";
    return false;
  }

  auto it_from_before = iterator_equal_or_before(stamp_from);
  auto it_to_after = iterator_equal_or_after(stamp_to);
  CHECK(it_from_before != buffer_.end());
  CHECK(it_to_after != buffer_.end());
  auto it_from_after = it_from_before;
  ++it_from_after;
  auto it_to_before = it_to_after;
  --it_to_before;
  if(it_from_after == it_to_before)
  {
    LOG(WARNING) << "Not enough data for interpolation";
    return false;
  }

  // Count number measurements.
  size_t n = 0;
  auto it = it_from_after;
  while(it != it_to_after)
  {
    ++n;
    ++it;
  }
  n += 2;

  // Interpolate values at start and end and copy in output vector.
  stamps->resize(1, n);
  values->resize(kDim, n);
  for(size_t i = 0; i < n; ++i)
  {
    if(i == 0)
    {
      (*stamps)(i) = stamp_from;
      const double w =
          static_cast<double>(stamp_from - it_from_before->first) /
          static_cast<double>(it_from_after->first - it_from_before->first);
      values->col(i) = (1.0 - w) * it_from_before->second + w * it_from_after->second;
    }
    else if(i == n-1)
    {
      (*stamps)(i) = stamp_to;
      const double w =
          static_cast<double>(stamp_to - it_to_before->first) /
          static_cast<double>(it_to_after->first - it_to_before->first);
      values->col(i) = (1.0 - w) * it_to_before->second + w * it_to_after->second;
    }
    else
    {
      (*stamps)(i) = it_from_after->first;
      values->col(i) = it_from_after->second;
      ++it_from_after;
    }
  }
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
