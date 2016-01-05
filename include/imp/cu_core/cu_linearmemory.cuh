#ifndef IMP_CU_LINEARHOSTMEMORY_CUH
#define IMP_CU_LINEARHOSTMEMORY_CUH

//#include <stdio.h>
//#include <assert.h>
//#include <cstdlib>
#include <memory>

#include <imp/core/linearmemory_base.hpp>
#include <imp/core/linearmemory.hpp>
#include <imp/core/pixel.hpp>
#include <imp/cu_core/cu_memory_storage.cuh>
#include <imp/cu_core/cu_pixel_conversion.hpp>

namespace imp {
namespace cu {

template<typename Pixel>
class LinearMemory : public LinearMemoryBase
{
public:
  using Memory = imp::cu::MemoryStorage<Pixel>;
  using Deallocator = imp::cu::MemoryDeallocator<Pixel>;

public:
  __host__ LinearMemory() = delete;
  virtual ~LinearMemory() { }

  __host__ LinearMemory(const std::uint32_t& length);
  __host__ LinearMemory(const imp::cu::LinearMemory<Pixel>& from);
  __host__ LinearMemory(const imp::LinearMemory<Pixel>& from);
//  __host__ LinearMemory(Pixel* host_data, const std::uint32_t& length,
//                        bool use_ext_data_pointer = false);

  /**
   * @brief Returns a pointer to the device buffer.
   */
  Pixel* data();
  const Pixel* data() const;

  /** Returns the buffer as respective cuda vector type that is pointing to the beginning for the data buffer.
   * @note this is mainly for convenience when calling cuda functions / kernels.
   */
  auto cuData() -> decltype(imp::cu::toCudaVectorType(this->data()));
  auto cuData() const -> decltype(imp::cu::toConstCudaVectorType(this->data()));

  /** Sets a certain value to all pixels in the data vector.
   */
  void setValue(const Pixel& value);

  /** Copy data to a host class instance.
   */
  void copyTo(imp::cu::LinearMemory<Pixel>& dst);

  /** Copy data from a host class instance.
   */
  void copyFrom(const imp::cu::LinearMemory<Pixel>& dst);

  /** Copy data to a host class instance.
   */
  void copyTo(imp::LinearMemory<Pixel>& dst);

  /** Copy data from a host class instance.
   */
  void copyFrom(const imp::LinearMemory<Pixel>& dst);

  /** Returns the total amount of bytes saved in the data buffer. */
  virtual size_t bytes() const override { return this->length()*sizeof(Pixel); }

  /** Returns the total amount of bytes for the region-of-interest saved in the data buffer. */
  virtual size_t roiBytes() const override { return this->roi().length()*sizeof(Pixel); }


  /** Returns the bit depth of the data pointer. */
  virtual std::uint8_t bitDepth() const override { return 8*sizeof(Pixel); }

  /** Returns flag if the image data resides on the device/GPU (TRUE) or host/GPU (FALSE) */
  virtual bool isGpuMemory() const  override { return true; }

private:
  std::unique_ptr<Pixel, Deallocator> data_;
};

// convenience typedefs
// (sync with explicit template class instantiations at the end of the cpp file)
typedef LinearMemory<imp::Pixel8uC1> LinearMemory8uC1;
typedef LinearMemory<imp::Pixel8uC2> LinearMemory8uC2;
typedef LinearMemory<imp::Pixel8uC3> LinearMemory8uC3;
typedef LinearMemory<imp::Pixel8uC4> LinearMemory8uC4;

typedef LinearMemory<imp::Pixel16uC1> LinearMemory16uC1;
typedef LinearMemory<imp::Pixel16uC2> LinearMemory16uC2;
typedef LinearMemory<imp::Pixel16uC3> LinearMemory16uC3;
typedef LinearMemory<imp::Pixel16uC4> LinearMemory16uC4;

typedef LinearMemory<imp::Pixel32uC1> LinearMemory32uC1;
typedef LinearMemory<imp::Pixel32uC2> LinearMemory32uC2;
typedef LinearMemory<imp::Pixel32uC3> LinearMemory32uC3;
typedef LinearMemory<imp::Pixel32uC4> LinearMemory32uC4;

typedef LinearMemory<imp::Pixel32sC1> LinearMemory32sC1;
typedef LinearMemory<imp::Pixel32sC2> LinearMemory32sC2;
typedef LinearMemory<imp::Pixel32sC3> LinearMemory32sC3;
typedef LinearMemory<imp::Pixel32sC4> LinearMemory32sC4;

typedef LinearMemory<imp::Pixel32fC1> LinearMemory32fC1;
typedef LinearMemory<imp::Pixel32fC2> LinearMemory32fC2;
typedef LinearMemory<imp::Pixel32fC3> LinearMemory32fC3;
typedef LinearMemory<imp::Pixel32fC4> LinearMemory32fC4;

} // namespace cu
} // namespace imp

#endif // IMP_LINEARHOSTMEMORY_H
