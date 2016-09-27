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

#include <iostream>
#include <cuda_runtime_api.h>
#include <imp/core/types.hpp>
#include <imp/core/pixel_enums.hpp>
#include <imp/core/size.hpp>
#include <ze/common/logging.hpp>

namespace ze {
namespace cu {

template <typename Pixel, ze::PixelType pixel_type = ze::PixelType::undefined>
struct MemoryStorage
{
public:
  // we don't construct an instance but only use it for the static functions
  // (at least for the moment)
  MemoryStorage() = delete;
  virtual ~MemoryStorage() = delete;

  //! @todo (MWE) do we wanna have a init flag for device memory?
  static Pixel* alloc(const uint32_t num_elements)
  {
    CHECK_GT(num_elements, 0u);

    const uint32_t memory_size = sizeof(Pixel) * num_elements;
    //std::cout << "cu::MemoryStorage::alloc: memory_size=" << memory_size << "; sizeof(Pixel)=" << sizeof(Pixel) << std::endl;

    Pixel* p_data = nullptr;
    cudaError_t cu_err = cudaMalloc((void**)&p_data, memory_size);

    CHECK_NE(cu_err, cudaErrorMemoryAllocation);
    CHECK_EQ(cu_err, cudaSuccess);

    return p_data;
  }

  /**
   * @brief alignedAlloc allocates an aligned 2D memory block (\a width x \a height) on the GPU (CUDA)
   * @param width Image width
   * @param height Image height
   * @param pitch Row alignment [bytes]
   *
   */
  static Pixel* alignedAlloc(const uint32_t width, const uint32_t height,
                             uint32_t* pitch)
  {
    CHECK_GT(width, 0u);
    CHECK_GT(height, 0u);

    uint32_t width_bytes = width * sizeof(Pixel);
    //std::cout << "width_bytes: " << width_bytes << std::endl;
    const int align_bytes = 1536;
    if (pixel_type == ze::PixelType::i8uC3 && width_bytes % align_bytes)
    {
      width_bytes += (align_bytes-(width_bytes%align_bytes));
    }
    //std::cout << "width_bytes: " << width_bytes << std::endl;

    size_t intern_pitch;
    Pixel* p_data = nullptr;
    cudaError_t cu_err = cudaMallocPitch((void **)&p_data, &intern_pitch,
                                         width_bytes, (uint32_t)height);

    *pitch = static_cast<uint32_t>(intern_pitch);
    //("pitch: %lu, i_pitch: %lu, width_bytes: %lu\n", *pitch, intern_pitch, width_bytes);

    CHECK_NE(cu_err, cudaErrorMemoryAllocation);
    CHECK_EQ(cu_err, cudaSuccess);

    return p_data;
  }


  /**
   * @brief alignedAlloc allocates an aligned 2D memory block of given \a size on the GPU (CUDA)
   * @param size Image size
   * @param pitch Row alignment [bytes]
   * @return
   */
  static Pixel* alignedAlloc(ze::Size2u size, uint32_t* pitch)
  {
    return alignedAlloc(size[0], size[1], pitch);
  }


  static void free(Pixel* buffer)
  {
    cudaFree(buffer);
  }

};


/**
 * @brief The Deallocator struct offers the ability to have custom deallocation methods.
 *
 * The Deallocator struct can be used as e.g. having custom deallocations with
 * shared pointers. Furthermore it enables the usage of external memory buffers
 * using shared pointers but not taking ownership of the memory. Be careful when
 * doing so as an application would behave badly if the memory got deleted although
 * we are still using it.
 *
 */
template<typename Pixel>
struct MemoryDeallocator
{
  // Default custom deleter assuming we use arrays (new PixelType[length])
  MemoryDeallocator()
    : f([](Pixel* p) { cudaFree(p); })
{ }

// allow us to define a custom deallocator
explicit MemoryDeallocator(std::function<void(Pixel*)> const &_f)
  : f(_f)
{ }

void operator()(Pixel* p) const
{
  f(p);
}

private:
std::function< void(Pixel* )> f;
};

} // namespace cu
} // namespace ze

