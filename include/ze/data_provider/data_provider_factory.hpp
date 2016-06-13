#pragma once

#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

DataProviderBase::Ptr loadDataProviderFromGflags(
    const std::uint32_t num_cams, const std::uint32_t num_imus);

} // namespace ze
