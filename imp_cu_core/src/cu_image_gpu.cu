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
#include <imp/cu_core/cu_image_gpu.cuh>
#include <iostream>
#include <memory>
#include <ze/common/logging.hpp>
#include <imp/cu_core/cu_linearmemory.cuh>
#include <imp/cu_core/cu_texture.cuh>
#include <imp/cu_core/cu_utils.hpp>
// kernel includes
#include <imp/cu_core/cu_k_setvalue.cuh>

namespace ze {
namespace cu {

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageGpu<Pixel>::ImageGpu(const ze::Size2u& size)
  : Base(size)
{
  data_.reset(Memory::alignedAlloc(this->size(), &this->header_.pitch));
  channel_format_desc_ = toCudaChannelFormatDesc(pixel_type<Pixel>::type);
  this->header_.memory_type = MemoryType::GpuAligned;
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageGpu<Pixel>::ImageGpu(const ImageGpu& from)
  : ImageGpu(from.size())
{
  this->copyFrom(from);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageGpu<Pixel>::ImageGpu(const Image<Pixel>& from)
  : ImageGpu(from.size())
{
  this->copyFrom(from);
}

////-----------------------------------------------------------------------------
//template<typename Pixel>
//ImageGpu<Pixel>
//::ImageGpu(Pixel* data, uint32_t width, uint32_t height,
//           uint32_t pitch, bool use_ext_data_pointer)
//  : Base(width, height)
//{
//  CHECK_NOT_NULL(data);
//
//  if(use_ext_data_pointer)
//  {
//    // This uses the external data pointer as internal data pointer.
//    auto dealloc_nop = [](Pixel*) { ; };
//    data_ = std::unique_ptr<Pixel, Deallocator>(
//          data, Deallocator(dealloc_nop));
//    this->pitch_ = pitch;
//  }
//  else
//  {
//    data_.reset(Memory::alignedAlloc(this->width(), this->height(), &this->pitch_));
//    size_t stride = pitch / sizeof(Pixel);

//    if (this->bytes() == pitch*height)
//    {
//      std::copy(data, data+stride*height, data_.get());
//    }
//    else
//    {
//      for (uint32_t y=0; y<height; ++y)
//      {
//        for (uint32_t x=0; x<width; ++x)
//        {
//          data_.get()[y*this->stride()+x] = data[y*stride + x];
//        }
//      }
//    }
//  }
//}

//-----------------------------------------------------------------------------
template<typename Pixel>
ImageGpu<Pixel>::~ImageGpu()
{
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void ImageGpu<Pixel>::copyTo(ze::Image<Pixel>& dst) const
{
  CHECK_EQ(this->size(), dst.size());
  cudaMemcpyKind memcpy_kind = dst.isGpuMemory() ? cudaMemcpyDeviceToDevice :
                                                   cudaMemcpyDeviceToHost;
  const cudaError cu_err = cudaMemcpy2D(dst.data(), dst.pitch(),
                                        this->data(), this->pitch(),
                                        this->rowBytes(),
                                        this->height(), memcpy_kind);
  CHECK_EQ(cu_err, ::cudaSuccess);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void ImageGpu<Pixel>::copyFrom(const Image<Pixel>& from)
{
  CHECK_EQ(this->size(), from.size());
  cudaMemcpyKind memcpy_kind = from.isGpuMemory() ? cudaMemcpyDeviceToDevice :
                                                    cudaMemcpyHostToDevice;
  const cudaError cu_err = cudaMemcpy2D(this->data(), this->pitch(),
                                        from.data(), from.pitch(),
                                        this->rowBytes(),
                                        this->height(), memcpy_kind);
  CHECK_EQ(cu_err, ::cudaSuccess);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* ImageGpu<Pixel>::data(uint32_t ox, uint32_t oy)
{
  CHECK_EQ(0u, ox) << "getting datapointer with offset not allowed for GPU memory";
  CHECK_EQ(0u, oy) << "getting datapointer with offset not allowed for GPU memory";
  return data_.get();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* ImageGpu<Pixel>::data(uint32_t ox, uint32_t oy) const
{
  CHECK_EQ(ox, 0u) << "Device memory offset access not possible.";
  CHECK_EQ(oy, 0u) << "Device memory offset access not possible.";
  return data_.get();
}

//-----------------------------------------------------------------------------
template<typename Pixel>
auto ImageGpu<Pixel>::cuData() -> decltype(ze::cu::toCudaVectorType(this->data()))
{
  return ze::cu::toCudaVectorType(this->data());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
auto ImageGpu<Pixel>::cuData() const -> decltype(ze::cu::toConstCudaVectorType(this->data()))
{
  return ze::cu::toConstCudaVectorType(this->data());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void ImageGpu<Pixel>::setValue(const Pixel& value)
{
  if (sizeof(Pixel) == 1)
  {
    cudaMemset2D((void*)this->data(), this->pitch(), (int)value.c[0], this->rowBytes(), this->height());
  }
  else
  {
    // fragmentation
    cu::Fragmentation<> frag(this->size());

    // todo add roi to kernel!
    ze::cu::k_setValue
        <<< frag.dimGrid, frag.dimBlock >>> (this->data(), this->stride(), value,
                                             this->width(), this->height());
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
std::shared_ptr<Texture2D> ImageGpu<Pixel>::genTexture(
    bool normalized_coords,
    cudaTextureFilterMode filter_mode,
    cudaTextureAddressMode address_mode,
    cudaTextureReadMode read_mode) const
{
  // don't blame me for doing a const_cast as binding textures needs a void* but
  // we want genTexture to be a const function as we don't modify anything here!
  return std::make_shared<Texture2D>(this->cuData(), this->pitch(), channel_format_desc_, this->size(),
                                     normalized_coords, filter_mode, address_mode, read_mode);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
//template<typename T>
ImageGpu<Pixel>& ImageGpu<Pixel>::operator*=(const Pixel& rhs)
{
  // fragmentation
  cu::Fragmentation<> frag(this->size());

  // todo add roi to kernel!
  ze::cu::k_pixelWiseMul
      <<< frag.dimGrid, frag.dimBlock >>> (this->data(), this->stride(), rhs,
                                           this->width(), this->height());
  return *this;

}

//=============================================================================
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class ImageGpu<ze::Pixel8uC1>;
template class ImageGpu<ze::Pixel8uC2>;
// be careful with 8uC3 images as pitch values are not divisable by 3!
template class ImageGpu<ze::Pixel8uC3>;
template class ImageGpu<ze::Pixel8uC4>;

template class ImageGpu<ze::Pixel16uC1>;
template class ImageGpu<ze::Pixel16uC2>;
template class ImageGpu<ze::Pixel16uC3>;
template class ImageGpu<ze::Pixel16uC4>;

template class ImageGpu<ze::Pixel32sC1>;
template class ImageGpu<ze::Pixel32sC2>;
template class ImageGpu<ze::Pixel32sC3>;
template class ImageGpu<ze::Pixel32sC4>;

template class ImageGpu<ze::Pixel32fC1>;
template class ImageGpu<ze::Pixel32fC2>;
template class ImageGpu<ze::Pixel32fC3>;
template class ImageGpu<ze::Pixel32fC4>;

} // namespace cu
} // namespace ze
