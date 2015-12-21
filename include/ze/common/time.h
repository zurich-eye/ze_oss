#pragma once

#include <ze/common/types.h>

namespace ze {

// Utilities for working with timestamps.
//
// Important: Always store int64_t nanosecond timestamps! We use signed type
//            to avoid errors when taking differences and we use nanoseconds
//            when saving to file to have a unique type for lookups in
//            dictionaries/maps.
namespace time {

// Seconds to nanoseconds.
inline constexpr int64_t secToNanosec(double seconds)
{
  return static_cast<int64_t>(seconds * 1e9);
}

// Milliseconds to nanoseconds.
inline constexpr int64_t millisecToNanosec(double milliseconds)
{
  return static_cast<int64_t>(milliseconds * 1e6);
}

// Nanoseconds to seconds.
inline constexpr double nanosecToSec(int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / static_cast<double>(1e9);
}

// Nanoseconds to milliseconds.
inline constexpr double nanosecToMillisec(int64_t nanoseconds)
{
  return static_cast<double>(nanoseconds) / static_cast<double>(1e6);
}

} // namespace time
} // namespace ze
