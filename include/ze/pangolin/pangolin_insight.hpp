#pragma once

#include <ze/pangolin/pangolin.hpp>
#include <ze/pangolin/type_watches.hpp>

//! Wrapping the type watches in new names for potential later extension.
//! Currently the type wrappers have the logging callbacks integrated to avoid
//! duplicating all operators on the child type.

namespace ze {
namespace internal {

template<typename Scalar>
using PangolinInsightWatch = PrimitiveTypeWrapperImpl<Scalar>;

}
}

//! Watch a class member variable.
#define PANGOLIN_WATCH(TYPE, MEMBER_NAME)                             \
  ze::internal::PangolinInsightWatch<TYPE> MEMBER_NAME =              \
    ze::internal::PangolinInsightWatch<TYPE>(#MEMBER_NAME)

#define PANGOLIN_WATCH_EXPR(VALUE, TYPE, NAME)                        \
  ze::PangolinPlotter::instance()->log<TYPE>(#NAME, VALUE)
