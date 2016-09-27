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

#include <cstdint>
#include <imp/core/size.hpp>
#include <imp/core/roi.hpp>

namespace ze {

/** \brief LinearMemory Base class for linear memory classes.
  */
class LinearMemoryBase
{
protected:
  LinearMemoryBase() = delete;

  LinearMemoryBase(const LinearMemoryBase& from)
    : size_(from.size_)
    , roi_(from.roi_)
  { }

  LinearMemoryBase(const uint32_t& length)
    : size_(length)
    , roi_(0, length)
  { }


public:
  virtual ~LinearMemoryBase()
  { }

  /** Returns the number of elements available in the internal buffer. */
  uint32_t length() const
  {
    return size_.length();
  }

  inline ze::Size1u size() const { return size_; }
  inline ze::Roi1u roi() const { return roi_; }

  /** Sets a region-of-interest (clamps towards boundaries [0, length[) */
  void setRoi(const ze::Roi1u& roi)
  {
    roi_.x() = std::max(0u, std::min(this->length()-1, roi.x()));
    uint32_t remaining_elements = this->length()-roi_.x();
    roi_.length() = std::max(1u, std::min(remaining_elements, roi.length()));
  }

  void resetRoi()
  {
    roi_ = ze::Roi1u(size_);
  }

  /** Returns the total amount of bytes saved in the data buffer. */
  virtual size_t bytes() const = 0;

  /** Returns the total amount of bytes for the region-of-interest saved in the data buffer. */
  virtual size_t roiBytes() const = 0;

  /** Returns the bit depth of the data pointer. */
  virtual std::uint8_t bitDepth() const = 0;

  /** Returns flag if the image data resides on the GPU (TRUE) or CPU (FALSE) */
  virtual bool isGpuMemory() const = 0;

protected:
  ze::Size1u size_;
  ze::Roi1u roi_;

};

} // namespace ze

