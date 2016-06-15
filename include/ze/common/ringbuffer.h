#pragma once

#include <map>
#include <tuple>
#include <thread>
#include <utility>
#include <mutex>
#include <Eigen/Dense>
#include <ze/common/logging.hpp>
#include <ze/common/ring_view.h>

#include <ze/common/types.h>
#include <ze/common/time_conversions.h>

namespace ze {


//! @todo: move the interpolators somewhere where they make more sense?
//!
//! Interpolators have to implement:
//! _ interpolate(Ringbuffer<...>::Ptr, int64_t time, Ringbuffer<...>timering_t::iterator*);
//! Passing the (optional) interator to the timestamp right before the to be
//! interpolated value speeds up the process.
//!
//! A nearest neighbour "interpolator".
struct InterpolatorNearest
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer, int64_t time);
};

//! A simple linear interpolator
struct InterpolatorLinear
{
  template<typename Ringbuffer_T>
  static typename Ringbuffer_T::DataType interpolate(
      Ringbuffer_T* buffer,
      int64_t time,
      typename Ringbuffer_T::timering_t::iterator* it_before_ptr = nullptr)
  {
    CHECK_EQ(false, it_before_ptr == nullptr) << "Non-initialized interpolation not"
                                          << "yet supported";
    auto it_before = *it_before_ptr;
    auto it_after = it_before + 1;

    const FloatType w1 =
        static_cast<FloatType>(time - *it_before) /
        static_cast<FloatType>(*it_after - *it_before);

    return (FloatType{1.0} - w1) * buffer->dataAtTimeIterator(it_before)
        + w1 * buffer->dataAtTimeIterator(it_after);
  }
};
using DefaultInterpolator = InterpolatorLinear;


//! a fixed size timed buffer templated on the number of entries
//! Opposed to the `Buffer`, values are expected to be received ORDERED in
//! TIME!
// Oldest entry: buffer.begin(), newest entry: buffer.rbegin()
template <typename Scalar, size_t ValueDim, size_t Size>
class Ringbuffer
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ringbuffer is friend with the interpolator types.
  friend struct InterpolatorNearest;
  friend struct InterpolatorLinear;

  typedef int64_t time_t;
  typedef Eigen::Matrix<time_t, Size, 1> times_t;
  typedef Eigen::Matrix<time_t, Eigen::Dynamic, 1> times_dynamic_t;
  typedef Eigen::Matrix<Scalar, ValueDim, Size> data_t;
  typedef Eigen::Matrix<Scalar, ValueDim, Eigen::Dynamic> data_dynamic_t;

  // time ring is used to keep track of the positions of the data
  // in the dataring
  // uses fixed size ring_view
  typedef ring_view<time_t> timering_t;

  using DataType = Eigen::Matrix<Scalar, ValueDim, 1>;
  using DataTypeMap = Eigen::Map<DataType>;

  // a series of return types
  using DataBoolPair = std::pair<DataType, bool>;
  using TimeDataBoolTuple = std::tuple<time_t, DataType, bool>;
  using TimeDataRangePair = std::pair<times_dynamic_t, data_dynamic_t >;

  Ringbuffer()
    : times_(timering_t(times_raw_.data(),
                        times_raw_.data() + Size,
                        times_raw_.data(),
                        0))
  {
  }
  // no copy, no move as there is no way to track the mutex
  Ringbuffer(const Ringbuffer& from) = delete;
  Ringbuffer(const Ringbuffer&& from) = delete;

  inline void insert(time_t stamp,
                     const DataType& data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    times_.push_back(stamp);
    data_.col(times_.back_idx()) = data;
  }

  //! Get value with timestamp closest to stamp. Boolean returns if successful.
  std::tuple<time_t, DataType, bool> getNearestValue(time_t stamp);

  //! Get oldest value in buffer.
  std::pair<DataType, bool> getOldestValue() const;

  //! Get newest value in buffer.
  std::pair<DataType, bool> getNewestValue() const;

  //! Get timestamps of newest and oldest entry.
  std::tuple<time_t, time_t, bool> getOldestAndNewestStamp() const;

  /*! @brief Get Values between timestamps.
   *
   * If timestamps are not matched, the values
   * are interpolated. Returns a vector of timestamps and a block matrix with
   * values as columns. Returns empty matrices if not successful.
   */
  template <typename Interpolator = DefaultInterpolator>
  std::pair<Eigen::Matrix<time_t, Eigen::Dynamic, 1>,
            Eigen::Matrix<Scalar, ValueDim, Eigen::Dynamic> >
  getBetweenValuesInterpolated(time_t stamp_from, time_t stamp_to);

  inline void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    times_.reset();
  }

  inline size_t size() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.size();
  }

  inline bool empty() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return times_.empty();
  }

  //! technically does not remove but only moves the beginning of the ring
  inline void removeDataBeforeTimestamp(time_t stamp)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    removeDataBeforeTimestamp_impl(stamp);
  }

  inline void removeDataOlderThan(FloatType seconds)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if(times_.empty())
    {
      return;
    }

    removeDataBeforeTimestamp_impl(
          times_.back() - secToNanosec(seconds));
  }

  inline void lock() const
  {
    mutex_.lock();
  }

  inline void unlock() const
  {
    mutex_.unlock();
  }

  const data_t& data() const
  {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return data_;
  }

  const timering_t& times() const
  {
    CHECK(!mutex_.try_lock()) << "Call lock() before accessing data.";
    return times_;
  }

  typename timering_t::iterator iterator_equal_or_before(time_t stamp);
  typename timering_t::iterator iterator_equal_or_after(time_t stamp);

  //! returns an iterator to the first element in the times_ ring that
  //! is greater or equal to stamp
  inline typename timering_t::iterator lower_bound(time_t stamp);

protected:
  mutable std::mutex mutex_;
  data_t data_;
  times_t times_raw_;
  timering_t times_;

  //! return the data at a given point in time
  inline DataType dataAtTimeIterator(typename timering_t::iterator iter) const
  {
    //! @todo: i believe this is wrong.
    return data_.col(iter.container_index());
  }

  //! return the data at a given point in time (const)
  inline DataType dataAtTimeIterator(typename timering_t::const_iterator iter) const
  {
    //! @todo: i believe this is wrong.
    return data_.col(iter.container_index());
  }

  //! shifts the starting point of the ringbuffer to the given timestamp
  //! no resizing or deletion happens.
  inline void removeDataBeforeTimestamp_impl(time_t stamp)
  {
    auto it = lower_bound(stamp);
    times_.reset_front(it.container_index());
  }
};

} // namespace ze

#include <ze/common/ringbuffer-inl.h>
