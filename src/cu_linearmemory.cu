#include <imp/cu_core/cu_linearmemory.cuh>

#include <imp/cu_core/cu_exception.hpp>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_k_setvalue.cuh>


namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const std::uint32_t& length)
  : LinearMemoryBase(length)
  , data_(Memory::alloc(this->length()))
{
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const ze::cu::LinearMemory<Pixel>& from)
  : ze::cu::LinearMemory<Pixel>(from.length())
{
  if (from.data() == 0)
  {
    throw ze::cu::Exception("'from' data not valid", __FILE__, __FUNCTION__, __LINE__);
  }
  this->copyFrom(from);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const ze::LinearMemory<Pixel>& from)
  : ze::cu::LinearMemory<Pixel>(from.length())
{
  if (from.data() == 0)
  {
    throw ze::cu::Exception("'from' data not valid", __FILE__, __FUNCTION__, __LINE__);
  }
  this->copyFrom(from);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* LinearMemory<Pixel>::data()
{
  return data_.get();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* LinearMemory<Pixel>::data() const
{
  return reinterpret_cast<const Pixel*>(data_.get());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
auto LinearMemory<Pixel>::cuData() -> decltype(ze::cu::toCudaVectorType(this->data()))
{
  return ze::cu::toCudaVectorType(this->data());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
auto LinearMemory<Pixel>::cuData() const -> decltype(ze::cu::toConstCudaVectorType(this->data()))
{
  return ze::cu::toConstCudaVectorType(this->data());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::setValue(const Pixel& value)
{
  if (sizeof(Pixel) == 1)
  {
    cudaMemset((void*)(this->data()+this->roi().x()), (int)value.c[0], this->roiBytes());
  }
  else
  {
    // fragmentation
    cu::Fragmentation<32,1> frag(this->roi().length());

    // todo add roi to kernel!
    ze::cu::k_setValue
        <<< frag.dimGrid, frag.dimBlock
        >>> (this->data(), this->roi().x(), this->roi().length(), value);
  }
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyTo(ze::cu::LinearMemory<Pixel>& dst)
{
  if (dst.data() == 0 || !data_)
    IMP_THROW_EXCEPTION("'from' or 'to' data is not valid");
  if (this->roiBytes() != dst.roiBytes())
    IMP_THROW_EXCEPTION("source and destination array region of interests are of different length (byte length checked)");

  const cudaError cu_err =
      cudaMemcpy(dst.data()+dst.roi().x(), this->data()+this->roi().x(),
                 this->roiBytes(), cudaMemcpyDeviceToDevice);

  if (cu_err != cudaSuccess)
    IMP_CU_THROW_EXCEPTION("cudaMemcpy returned error code", cu_err);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyFrom(const ze::cu::LinearMemory<Pixel>& from)
{
  if (from.data() == 0 || !data_)
    IMP_THROW_EXCEPTION("'from' or 'to' data is not valid");
  if (this->roiBytes() != from.roiBytes())
    IMP_THROW_EXCEPTION("source and destination array region of interests are of different length (byte length checked)");

  const cudaError cu_err =
      cudaMemcpy(this->data()+this->roi().x(), from.data()+from.roi().x(),
                 from.roiBytes(), cudaMemcpyDeviceToDevice);

  if (cu_err != cudaSuccess)
    IMP_CU_THROW_EXCEPTION("cudaMemcpy returned error code", cu_err);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyTo(ze::LinearMemory<Pixel>& dst)
{
  if (dst.data() == 0 || !data_)
    IMP_THROW_EXCEPTION("'from' or 'to' data is not valid");
  if (this->roiBytes() != dst.roiBytes())
    IMP_THROW_EXCEPTION("source and destination array region of interests are of different length (byte length checked)");

  const cudaError cu_err =
      cudaMemcpy(dst.data()+dst.roi().x(), this->data()+this->roi().x(),
                 this->roiBytes(), cudaMemcpyDeviceToHost);

  if (cu_err != cudaSuccess)
    IMP_CU_THROW_EXCEPTION("cudaMemcpy returned error code", cu_err);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyFrom(const ze::LinearMemory<Pixel>& from)
{
  if (from.data() == 0 || !data_)
    IMP_THROW_EXCEPTION("'from' or 'to' data is not valid");
  if (this->roiBytes() != from.roiBytes())
    IMP_THROW_EXCEPTION("source and destination array region of interests are of different length (byte length checked)");

  const cudaError cu_err =
      cudaMemcpy(this->data()+this->roi().x(), from.data()+from.roi().x(),
                 from.roiBytes(), cudaMemcpyHostToDevice);

  if (cu_err != cudaSuccess)
    IMP_CU_THROW_EXCEPTION("cudaMemcpy returned error code", cu_err);
}


//=============================================================================
// Explicitely instantiate the desired classes
template class LinearMemory<ze::Pixel8uC1>;
template class LinearMemory<ze::Pixel8uC2>;
template class LinearMemory<ze::Pixel8uC3>;
template class LinearMemory<ze::Pixel8uC4>;

template class LinearMemory<ze::Pixel16uC1>;
template class LinearMemory<ze::Pixel16uC2>;
template class LinearMemory<ze::Pixel16uC3>;
template class LinearMemory<ze::Pixel16uC4>;

template class LinearMemory<ze::Pixel32uC1>;
template class LinearMemory<ze::Pixel32uC2>;
template class LinearMemory<ze::Pixel32uC3>;
template class LinearMemory<ze::Pixel32uC4>;

template class LinearMemory<ze::Pixel32sC1>;
template class LinearMemory<ze::Pixel32sC2>;
template class LinearMemory<ze::Pixel32sC3>;
template class LinearMemory<ze::Pixel32sC4>;

template class LinearMemory<ze::Pixel32fC1>;
template class LinearMemory<ze::Pixel32fC2>;
template class LinearMemory<ze::Pixel32fC3>;
template class LinearMemory<ze::Pixel32fC4>;

} // namespace cu
} // namespace ze
