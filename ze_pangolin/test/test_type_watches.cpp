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

#include <iostream>
#include <string>

#include <ze/common/test_entrypoint.hpp>
#include <ze/pangolin/type_watches.hpp>

#define ASSERT_OPERATOR(TYPE, LHS, RHS, OP)       \
{                                                 \
  PrimitiveTypeWrapperImpl<TYPE> wrappedLhs(LHS, false); \
  PrimitiveTypeWrapperImpl<TYPE> wrappedRhs(RHS, false); \
  TYPE plainLhs = LHS;                            \
  TYPE plainRhs = RHS;                            \
  TYPE result = plainLhs OP plainRhs;             \
  EXPECT_EQ(wrappedLhs OP wrappedRhs, result);    \
  EXPECT_EQ(wrappedLhs OP plainRhs, result);      \
  EXPECT_EQ(plainLhs OP wrappedRhs, result);      \
}

#define ASSERT_OPERATOR_TYPE_INT(TYPE)      \
  ASSERT_OPERATOR(TYPE, 10, 5, +);        \
  ASSERT_OPERATOR(TYPE, 10, 5, -);        \
  ASSERT_OPERATOR(TYPE, 10, 5, *);        \
  ASSERT_OPERATOR(TYPE, 10, 5, /);        \
  ASSERT_OPERATOR(TYPE, 10, 5, %);        \
  ASSERT_OPERATOR(TYPE, 5, 10, %);        \
  ASSERT_OPERATOR(TYPE, 10, 5, &);        \
  ASSERT_OPERATOR(TYPE, 10, 5, |);        \
  ASSERT_OPERATOR(TYPE, 10, 5, ^);        \
  ASSERT_OPERATOR(TYPE, 10, 5, |);

#define ASSERT_OPERATOR_TYPE_DOUBLE(TYPE)      \
  ASSERT_OPERATOR(TYPE, 10, 5, +);           \
  ASSERT_OPERATOR(TYPE, 10, 5, -);           \
  ASSERT_OPERATOR(TYPE, 10, 5, *);           \
  ASSERT_OPERATOR(TYPE, 10, 5, /);

TEST(TypeWatchesTest, testAllTypes)
{
  using namespace ze;

  ASSERT_OPERATOR_TYPE_DOUBLE(double);
  ASSERT_OPERATOR_TYPE_DOUBLE(float);
  ASSERT_OPERATOR_TYPE_DOUBLE(long double);

  ASSERT_OPERATOR_TYPE_INT(int);
  ASSERT_OPERATOR_TYPE_INT(unsigned);
  ASSERT_OPERATOR_TYPE_INT(short);
  ASSERT_OPERATOR_TYPE_INT(char);
  ASSERT_OPERATOR_TYPE_INT(unsigned long long);
}

ZE_UNITTEST_ENTRYPOINT
