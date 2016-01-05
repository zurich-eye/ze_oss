#include <imp/cu_core/cu_linearmemory.cuh>

#include <imp/cu_core/cu_exception.hpp>
#include <imp/cu_core/cu_utils.hpp>
#include <imp/cu_core/cu_k_setvalue.cuh>


namespace imp {
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
LinearMemory<Pixel>::LinearMemory(const imp::cu::LinearMemory<Pixel>& from)
  : imp::cu::LinearMemory<Pixel>(from.length())
{
  if (from.data() == 0)
  {
    throw imp::cu::Exception("'from' data not valid", __FILE__, __FUNCTION__, __LINE__);
  }
  this->copyFrom(from);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const imp::LinearMemory<Pixel>& from)
  : imp::cu::LinearMemory<Pixel>(from.length())
{
  if (from.data() == 0)
  {
    throw imp::cu::Exception("'from' data not valid", __FILE__, __FUNCTION__, __LINE__);
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
auto LinearMemory<Pixel>::cuData() -> decltype(imp::cu::toCudaVectorType(this->data()))
{
  return imp::cu::toCudaVectorType(this->data());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
auto LinearMemory<Pixel>::cuData() const -> decltype(imp::cu::toConstCudaVectorType(this->data()))
{
  return imp::cu::toConstCudaVectorType(this->data());
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
    imp::cu::k_setValue
        <<< frag.dimGrid, frag.dimBlock
        >>> (this->data(), this->roi().x(), this->roi().length(), value);
  }
  IMP_CUDA_CHECK();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyTo(imp::cu::LinearMemory<Pixel>& dst)
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
void LinearMemory<Pixel>::copyFrom(const imp::cu::LinearMemory<Pixel>& from)
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
void LinearMemory<Pixel>::copyTo(imp::LinearMemory<Pixel>& dst)
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
void LinearMemory<Pixel>::copyFrom(const imp::LinearMemory<Pixel>& from)
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
template class LinearMemory<imp::Pixel8uC1>;
template class LinearMemory<imp::Pixel8uC2>;
template class LinearMemory<imp::Pixel8uC3>;
template class LinearMemory<imp::Pixel8uC4>;

template class LinearMemory<imp::Pixel16uC1>;
template class LinearMemory<imp::Pixel16uC2>;
template class LinearMemory<imp::Pixel16uC3>;
template class LinearMemory<imp::Pixel16uC4>;

template class LinearMemory<imp::Pixel32uC1>;
template class LinearMemory<imp::Pixel32uC2>;
template class LinearMemory<imp::Pixel32uC3>;
template class LinearMemory<imp::Pixel32uC4>;

template class LinearMemory<imp::Pixel32sC1>;
template class LinearMemory<imp::Pixel32sC2>;
template class LinearMemory<imp::Pixel32sC3>;
template class LinearMemory<imp::Pixel32sC4>;

template class LinearMemory<imp::Pixel32fC1>;
template class LinearMemory<imp::Pixel32fC2>;
template class LinearMemory<imp::Pixel32fC3>;
template class LinearMemory<imp::Pixel32fC4>;

} // namespace cu
} // namespace imp
