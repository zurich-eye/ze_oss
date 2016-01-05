#ifndef IMP_CV_CONNECTOR_PIXEL_TYPES
#define IMP_CV_CONNECTOR_PIXEL_TYPES

#include <imp/core/pixel_enums.hpp>

namespace imp {

/**
 * @brief pixelTypeFromCv converges OpenCV pixel type to IMP pixel types
 * @param type OpenCV pixel type (e.g. CV_8UC1 for single channel 8-bit pixels)
 * @return IMP pixel type (e.g. imp::PixelType::i8uC1 for single channel 8-bit pixels)
 */
imp::PixelType pixelTypeFromCv(int type);


/**
 * @brief pixelTypeFromCv converges IMP pixel types to OpenCV pixel type
 * @param type IMP pixel type (e.g. imp::PixelType::i8uC1 for single channel 8-bit pixels)
 * @return OpenCV pixel type (e.g. CV_8UC1 for single channel 8-bit pixels)
 */
int pixelTypeToCv(imp::PixelType type);


} // namespace imp

#endif // IMP_CV_CONNECTOR_PIXEL_TYPES

