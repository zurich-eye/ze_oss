#pragma once

#include <ze/pangolin/pangolin.hpp>
#include <ze/pangolin/type_watches.hpp>

//! Wrapping the type watches in new names for potential later extension.
//! Currently the type wrappers have the logging callbacks integrated to avoid
//! duplicating all operators on the child type.

namespace ze {
namespace internal {

template<typename Scalar>
using PangolinInsight = PrimitiveTypeWrapperImpl<Scalar>;

}
}

//! Watch a class member variable.
#define PANGOLIN_WATCH(TYPE, MEMBER_NAME)                         \
  ze::internal::PangolinInsight<TYPE> MEMBER_NAME =               \
    ze::internal::PangolinInsight<TYPE>(#MEMBER_NAME)
