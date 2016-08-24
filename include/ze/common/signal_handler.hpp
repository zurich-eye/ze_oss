// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#pragma once

#include <ze/common/noncopyable.hpp>

namespace ze {

/**
 * RAII-style signal handler that simply clears a flag when receiving SIGHUP,
 * SIGINT or SIGTERM
 **/
class SimpleSigtermHandler : Noncopyable
{
public:
  SimpleSigtermHandler(volatile bool& flag);
  ~SimpleSigtermHandler();
};

} // namespace ze
