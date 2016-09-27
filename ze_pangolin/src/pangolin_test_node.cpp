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

#include <ze/pangolin/pangolin.hpp>
#include <ze/common/types.hpp>
#include <ze/pangolin/type_watches.hpp>
#include <ze/pangolin/pangolin_insight.hpp>

class Sample
{
public:
  void increment() { ++value; --value_another; }
private:
  //int value;
  PANGOLIN_WATCH(int, value);
  PANGOLIN_WATCH(int, value_another);
};

int main( int /*argc*/, char* argv[] )
{
//  PANGOLIN_WATCH(int, test);

//  int x = 2;

//  test = x;

//  while(true)
//  {
//    test += 0.3;
//    sleep(1u);
//  }

  std::shared_ptr<Sample> sample(std::make_shared<Sample>());
  ze::Vector3 vector; vector.setRandom();

  int i = 15;
  while(true)
  {
    sample->increment();
    PANGOLIN_WATCH_EXPR(i++, double, T);
    PANGOLIN_WATCH_EXPR(vector, ze::Vector3, vector);
    vector.setRandom();
    sleep(1u);
  }

  return 0;
}
