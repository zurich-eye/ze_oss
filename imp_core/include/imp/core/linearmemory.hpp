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

#include <stdio.h>
#include <assert.h>
#include <cstdlib>
#include <memory>

#include <imp/core/linearmemory_base.hpp>
#include <imp/core/memory_storage.hpp>
#include <imp/core/pixel.hpp>

namespace ze {

template<typename Pixel>
class LinearMemory : public LinearMemoryBase
{
public:
  using Memory = ze::MemoryStorage<Pixel>;
  using Deallocator = ze::MemoryDeallocator<Pixel>;

public:
  LinearMemory() = delete;
  virtual ~LinearMemory() { }
  LinearMemory(const uint32_t& length);
  LinearMemory(const LinearMemory<Pixel>& from);
  LinearMemory(Pixel* host_data, const uint32_t& length,
               bool use_ext_data_pointer = false);

  /**
   * @brief Returns a pointer to the device buffer.
   * @param[in] offset Offset of the pointer array.
   * @return Pointer to the device buffer.
   *
   * @note The pointer can be offset to position \a offset.
   *
   */
  Pixel* data(uint32_t offset = 0);

  /** Returns a const pointer to the device buffer.
   * @param[in] offset Desired offset within the array.
   * @return Const pointer to the device buffer.
   */
  const Pixel* data(uint32_t offset = 0) const;

  /** Sets a certain value to all pixels in the data vector.
   */
  void setValue(const Pixel& value);

  /** Copy data to another class instance.
   */
  void copyTo(LinearMemory<Pixel>& dst);

  //! @todo (MWE) operator= for copyTo/copyFrom?
  LinearMemory<Pixel>& operator=(Pixel rhs);

  /** Returns the total amount of bytes saved in the data buffer. */
  virtual size_t bytes() const override { return this->length()*sizeof(Pixel); }

  /** Returns the total amount of bytes for the region-of-interest saved in the data buffer. */
  virtual size_t roiBytes() const override { return this->roi().length()*sizeof(Pixel); }

  /** Returns the bit depth of the data pointer. */
  virtual std::uint8_t bitDepth() const override { return 8*sizeof(Pixel); }

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  virtual bool isGpuMemory() const  override { return false; }

  /** Pixel access with (idx). */
   inline Pixel& operator()(uint32_t idx) {return *this->data(idx);}
   /** Pixel access with [idx]. */
   inline Pixel& operator[](uint32_t idx) {return *this->data(idx);}
   inline const Pixel& operator[](uint32_t idx) const {return *this->data(idx);}

private:
  std::unique_ptr<Pixel, Deallocator> data_;

};

// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef LinearMemory<ze::Pixel8uC1> LinearMemory8uC1;
typedef LinearMemory<ze::Pixel8uC2> LinearMemory8uC2;
typedef LinearMemory<ze::Pixel8uC3> LinearMemory8uC3;
typedef LinearMemory<ze::Pixel8uC4> LinearMemory8uC4;

typedef LinearMemory<ze::Pixel16uC1> LinearMemory16uC1;
typedef LinearMemory<ze::Pixel16uC2> LinearMemory16uC2;
typedef LinearMemory<ze::Pixel16uC3> LinearMemory16uC3;
typedef LinearMemory<ze::Pixel16uC4> LinearMemory16uC4;

typedef LinearMemory<ze::Pixel32uC1> LinearMemory32uC1;
typedef LinearMemory<ze::Pixel32uC2> LinearMemory32uC2;
typedef LinearMemory<ze::Pixel32uC3> LinearMemory32uC3;
typedef LinearMemory<ze::Pixel32uC4> LinearMemory32uC4;

typedef LinearMemory<ze::Pixel32sC1> LinearMemory32sC1;
typedef LinearMemory<ze::Pixel32sC2> LinearMemory32sC2;
typedef LinearMemory<ze::Pixel32sC3> LinearMemory32sC3;
typedef LinearMemory<ze::Pixel32sC4> LinearMemory32sC4;

typedef LinearMemory<ze::Pixel32fC1> LinearMemory32fC1;
typedef LinearMemory<ze::Pixel32fC2> LinearMemory32fC2;
typedef LinearMemory<ze::Pixel32fC3> LinearMemory32fC3;
typedef LinearMemory<ze::Pixel32fC4> LinearMemory32fC4;



} // namespace ze

