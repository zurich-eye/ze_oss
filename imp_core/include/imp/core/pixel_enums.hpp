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

namespace ze {

/**
 * @brief The PixelType enum defines a pixel's bit depth and number of channels.
 */
enum class PixelType
{
  undefined = -1,
  i8uC1,  //!< interleaved, 8-bit unsigned, 1 channel
  i8uC2,  //!< interleaved, 8-bit unsigned, 2 channel
  i8uC3,  //!< interleaved, 8-bit unsigned, 3 channel
  i8uC4,  //!< interleaved, 8-bit unsigned, 4 channel
  i16sC1, //!< interleaved, 16-bit signed, 1 channel
  i16sC2, //!< interleaved, 16-bit signed, 2 channel
  i16sC3, //!< interleaved, 16-bit signed, 3 channel
  i16sC4, //!< interleaved, 16-bit signed, 4 channel
  i16uC1, //!< interleaved, 16-bit unsigned, 1 channel
  i16uC2, //!< interleaved, 16-bit unsigned, 2 channel
  i16uC3, //!< interleaved, 16-bit unsigned, 3 channel
  i16uC4, //!< interleaved, 16-bit unsigned, 4 channel
  i32uC1, //!< interleaved, 32-bit unsigned, 1 channel
  i32uC2, //!< interleaved, 32-bit unsigned, 2 channel
  i32uC3, //!< interleaved, 32-bit unsigned, 3 channel
  i32uC4, //!< interleaved, 32-bit unsigned, 4 channel
  i32sC1, //!< interleaved, 32-bit signed, 1 channel
  i32sC2, //!< interleaved, 32-bit signed, 2 channel
  i32sC3, //!< interleaved, 32-bit signed, 3 channel
  i32sC4, //!< interleaved, 32-bit signed, 4 channel
  i32fC1, //!< interleaved, 32-bit float, 1 channel
  i32fC2, //!< interleaved, 32-bit float, 2 channel
  i32fC3, //!< interleaved, 32-bit float, 3 channel
  i32fC4  //!< interleaved, 32-bit float, 4 channel
};

/**
 * @brief The PixelOrder enum defines a pixel's channel ordering (for interleaved pixels).
 */
enum class PixelOrder
{
  undefined = -1,
  gray,  //!< single-channel grayscale
  rgb,   //!< 3-channel RGB
  bgr,   //!< 3-channel BGR
  rgba,  //!< 3-channel RGBA
  bgra   //!< 3-channel BGRA
};


} // namespace ze

