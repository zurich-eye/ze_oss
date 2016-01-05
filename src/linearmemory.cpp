#include <imp/core/linearmemory.hpp>

#include <cstring>
#include <algorithm>

#include <imp/core/exception.hpp>


namespace imp {

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const std::uint32_t& length)
  : LinearMemoryBase(length)
  , data_(Memory::alignedAlloc(this->length()))
{
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const LinearMemory<Pixel>& from)
  : LinearMemoryBase(from)
{
  if (from.data_ == 0)
  {
    throw imp::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
  }
  data_.reset(Memory::alignedAlloc(this->length()));
  std::copy(from.data_.get(), from.data_.get()+from.length(), data_.get());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(Pixel* host_data,
                                  const std::uint32_t& length,
                                  bool use_ext_data_pointer)
  : LinearMemoryBase(length)
{
  if (host_data == nullptr)
  {
    throw imp::Exception("input data not valid", __FILE__, __FUNCTION__, __LINE__);
  }

  if(use_ext_data_pointer)
  {
    // This uses the external data pointer and stores it as a 'reference':
    // memory won't be managed by us!
    auto dealloc_nop = [](Pixel*) { ; };
    data_ = std::unique_ptr<Pixel, Deallocator>(
          host_data, Deallocator(dealloc_nop));
  }
  else
  {
    // allocates an internal data pointer and copies the external data it.
    data_.reset(Memory::alignedAlloc(this->length()));
    std::copy(host_data, host_data+length, data_.get());
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* LinearMemory<Pixel>::data(std::uint32_t offset)
{
  if (offset > this->length())
  {
    throw imp::Exception("offset not in range", __FILE__, __FUNCTION__, __LINE__);
  }

  return &(data_.get()[offset]);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* LinearMemory<Pixel>::data(std::uint32_t offset) const
{
  if (offset > this->length())
  {
    throw imp::Exception("offset not in range", __FILE__, __FUNCTION__, __LINE__);
  }
  return reinterpret_cast<const Pixel*>(&(data_.get()[offset]));
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::setValue(const Pixel& value)
{
  std::fill(data_.get()+roi_.x(),
            data_.get()+this->roi().x()+this->roi().length(),
            value);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyTo(LinearMemory<Pixel>& dst)
{
  if (this->length() != dst.length())
  {
    throw imp::Exception("source and destination array are of different length", __FILE__, __FUNCTION__, __LINE__);
  }
  std::copy(data_.get(), data_.get()+this->length(), dst.data_.get());
}


//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>& LinearMemory<Pixel>::operator=(Pixel rhs)
{
  this->setValue(rhs);
  return *this;
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

} // namespace imp
