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
#include <gtest/gtest.h>

// system includes
#include <assert.h>
#include <cstdint>
#include <iostream>

#include <imp/core/size.hpp>

TEST(IMPCoreTestSuite,testSize)
{
  {
    ze::Size1u sz1u;
    EXPECT_EQ(0u, sz1u.length());
    uint32_t len=101;
    ze::Size1u sz(len);
    EXPECT_EQ(len, sz.length());
    EXPECT_EQ(len, sz.data()[0]);
    EXPECT_EQ(len, sz);
  }
  {
    ze::Size2u sz2u;
    EXPECT_EQ(0u, sz2u.width());
    EXPECT_EQ(0u, sz2u.height());

    // 2D sizes
    const std::int32_t w=10, h=13;
    const std::int32_t area = w*h;

    ze::Size2i sz(w,h);
    EXPECT_EQ(w, sz.width());
    EXPECT_EQ(h, sz.height());

    EXPECT_EQ(w, sz.data()[0]);
    EXPECT_EQ(h, sz.data()[1]);

    EXPECT_EQ(area, sz.area());
    EXPECT_EQ(area, sz.prod());

    // comparison operator tests
    ze::Size2i a(123,456);
    ze::Size2i b(123,456);
    ze::Size2i c(124,456);
    ze::Size2i d(124,457);

    ASSERT_TRUE((a == b));
    ASSERT_FALSE((a != b));
    ASSERT_FALSE((a == c));
    ASSERT_TRUE((a != c));
  }
}
