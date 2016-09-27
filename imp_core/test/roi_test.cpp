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

#include <imp/core/roi.hpp>


TEST(IMPCoreTestSuite,roiTest)
{
  //
  // 1D case uint
  //
  {
    uint32_t x=1, len=10;
    ze::Roi1u roi(x,len);
    ASSERT_TRUE(x == roi.x());
    ASSERT_TRUE(len == roi.length());

    ASSERT_TRUE(x == roi.lu()[0]);
    ASSERT_TRUE(len == roi.size()[0]);
  }

  //
  // 2D case
  //
  {
    std::int32_t x=1, y=2, w=10, h=13;
    ze::Roi2i roi(x,y,w,h);
    ASSERT_TRUE(x == roi.x());
    ASSERT_TRUE(y == roi.y());
    ASSERT_TRUE(w == roi.width());
    ASSERT_TRUE(h == roi.height());

    ASSERT_TRUE(x == roi.lu()[0]);
    ASSERT_TRUE(y == roi.lu()[1]);
    ASSERT_TRUE(w == roi.size()[0]);
    ASSERT_TRUE(h == roi.size()[1]);
  }

  // operator==, etc.
  {
    uint32_t x=1, y=2, w=10, h=13;
    ze::Roi2u roi1(x,y,w,h);
    ze::Roi2u roi2(x,y,w,h);
    ze::Roi2u roi3(x+1,y,w,h);

    ASSERT_TRUE(roi1 == roi2);
    ASSERT_FALSE(roi1 != roi2);
    ASSERT_FALSE(roi1 == roi3);
    ASSERT_TRUE(roi1 != roi3);
  }
}
