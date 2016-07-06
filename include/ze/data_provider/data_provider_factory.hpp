#pragma once

#include <gflags/gflags.h>
#include <ze/data_provider/data_provider_base.hpp>

DECLARE_uint64(num_imus);

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(const std::uint32_t num_cams);

} // namespace ze
