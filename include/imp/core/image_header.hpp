#pragma once

#include <ze/common/types.h>


namespace ze {

//! Defines an image header including all size information. No data included.
struct ImageHeader
{
  PixelType pixel_type{PixelType::undefined};
  size_t pixel_size{0}; //!< Pixel size in bytes.
  PixelOrder pixel_order{PixelOrder::undefined};
  Size2u size{0, 0};
  Roi2u roi{0, 0, 0, 0}; //!< Region of interest. x,y offset and width, height.
  size_t pitch{0}; //!< Row alignment in bytes.
  bool memory_type{MemoryType::Undefined}; //!< Memory Type.

  ImageHeader() = default;
  ~ImageHeader() = default;

  ImageHeader(
      PixelType _pixel_type,
      size_t _pixel_size = 0,
      PixelOrder _pixel_order,
      Size2u _size = Size2u{0,0},
      Roi2u _roi = Roi2u{0,0,0,0},
      size_t _pitch = 0,
      MemoryType _memory_type = MemoryType::Undefined)
    : pixel_type(_pixel_type)
    , pixel_size(_pixel_size)
    , pixel_order(_pixel_order)
    , size(_size)
    , roi(_roi)
    , pitch(_pitch)
    , memory_type(_memory_type)
  { ; }

  bool isGpuMemory()
  {
    switch (memory_type)
    {
    case MemoryType::Cpu:
    case MemoryType::CpuAligned:
      return false;
    case MemoryType::Managed:
    case MemoryType::Managed:
    case MemoryType::Unified:
    case MemoryType::UnifiedAligned:
      return true;
    defualt:
      CHECK(false) << "Undefined or unitialized memory";
      break;
    }
  }
};


} // namespace ze
