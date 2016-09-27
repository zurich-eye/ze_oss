// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
  ze::PangolinPlotter::instance().log<TYPE>(#NAME, VALUE)
