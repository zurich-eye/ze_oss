#ifndef IMP_MEMORY_STORAGE_HPP
#define IMP_MEMORY_STORAGE_HPP

#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <functional>
#include <algorithm>

#include <imp/core/exception.hpp>
#include <imp/core/size.hpp>
#include <imp/core/types.hpp>

namespace imp {

//--------------------------------------------------------------------------
template <typename Pixel, int memaddr_align=32, bool align_rows=true>
struct MemoryStorage
{
public:
  MemoryStorage() = delete;
  virtual ~MemoryStorage() = delete;

  /**
   * @brief alignedAlloc allocates an aligned block of memory
   * @param num_elements Number of (minimum) allocated elements
   * @param init_with_zeros Flag if the memory elements should be zeroed out (default=false).
   *
   * @note Internally we use the C11 function aligned_alloc although there
   *       are also alignment functions in C++11 but aligned_alloc is the only
   *       one where we don't have to mess around with allocated bigger chuncks of
   *       memory and shifting the start address accordingly. If you know a
   *       better approach using e.g. std::align(), let me know.
   */
  static Pixel* alignedAlloc(const size_t num_elements,
                             bool init_with_zeros=false)
  {
    if (num_elements == 0)
    {
      throw imp::Exception("Failed to allocate memory: num_elements=0");
    }

    // restrict the memory address alignment to be in the interval ]0,128] and
    // of power-of-two using the 'complement and compare' method
    assert((memaddr_align != 0) && memaddr_align <= 128 &&
           ((memaddr_align & (~memaddr_align + 1)) == memaddr_align));

    const size_t memory_size = sizeof(Pixel) * num_elements;
    //std::cout << "memory_size=" << memory_size << "; sizeof(Pixel)=" << sizeof(Pixel) << std::endl;

    // Pixel* p_data_aligned =
    //     (Pixel*)aligned_alloc(memaddr_align, memory_size);
    Pixel* p_data_aligned;
    int ret = posix_memalign((void**)&p_data_aligned, memaddr_align, memory_size);

    if (p_data_aligned == nullptr || ret != 0)
    {
      throw std::bad_alloc();
    }

    if (init_with_zeros)
    {
      std::fill(p_data_aligned, p_data_aligned+num_elements, Pixel(0));
    }

    return (Pixel*)p_data_aligned;
  }

  /**
   * @brief alignedAlloc allocates an aligned block that guarantees to host the image of size \a width \a x \a height
   * @param width Image width
   * @param height Image height
   * @param init_with_zeros Flag if the memory elements should be zeroed out (default=false).
   *
   * @note The allocator ensures that the starting adress of every row is aligned
   *       accordingly.
   *
   */
  static Pixel* alignedAlloc(const std::uint32_t width, const std::uint32_t height,
                             size_t* pitch, bool init_with_zeros=false)
  {
    if (width == 0 || height == 0)
    {
      throw imp::Exception("Failed to allocate memory: width or height is zero");
    }

    // restrict the memory address alignment to be in the interval ]0,128] and
    // of power-of-two using the 'complement and compare' method
    assert((memaddr_align != 0) && memaddr_align <= 128 &&
           ((memaddr_align & (~memaddr_align + 1)) == memaddr_align));

    // check if the width allows a correct alignment of every row, otherwise add padding
    const size_t width_bytes = width * sizeof(Pixel);
    // bytes % memaddr_align = 0 for bytes=n*memaddr_align is the reason for
    // the decrement in the following compution:
    const size_t bytes_to_add = (memaddr_align-1) - ((width_bytes-1) % memaddr_align);
    const std::uint32_t pitched_width = width + bytes_to_add/sizeof(Pixel);
    *pitch = width_bytes + bytes_to_add;
    return alignedAlloc(pitched_width*height, init_with_zeros);
  }

  /**
   * @brief alignedAlloc allocates an aligned block of memory that guarantees to host the image of size \a size
   * @param size Image size
   * @param pitch Row alignment [bytes] if padding is needed.
   * @param init_with_zeros Flag if the memory elements should be zeroed out (default=false).
   * @return
   */
  static Pixel* alignedAlloc(imp::Size2u size, size_t* pitch,
                             bool init_with_zeros=false)
  {
    return alignedAlloc(size[0], size[1], pitch, init_with_zeros);
  }


  /**
   * @brief free releases the pixel \a buffer
   * @param buffer
   */
  static void free(Pixel* buffer)
  {
    free(buffer);
  }
}; // struct MemoryStorage

/**
 * @brief The MemoryDeallocator struct offers the ability to have custom deallocation methods.
 *
 * The Deallocator struct can be used as e.g. having custom deallocations with
 * shared pointers. Furthermore it enables the usage of external memory buffers
 * using shared pointers but not taking ownership of the memory. Be careful when
 * doing so as an application would behave badly if the memory got deleted although
 * we are still using it.
 *
 */
template<typename Pixel>
class MemoryDeallocator
{
public:
  // Default custom deleter assuming we use arrays (new PixelType[length])
  MemoryDeallocator()
  {
    f_ = [](Pixel* p) {free(p);};
  }

  // allow us to define a custom deallocator
  explicit MemoryDeallocator(std::function<void(Pixel*)> const &f)
    : f_(f)
  { }

  void operator()(Pixel* p) const
  {
    f_(p);
  }

private:
  std::function< void(Pixel* )> f_;
}; // MemoryDeallocator

} // imp

#endif // IMAGE_ALLOCATOR_HPP
