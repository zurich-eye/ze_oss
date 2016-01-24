#include <ze/cameras/camera_impl.h>

namespace ze {


//-----------------------------------------------------------------------------
// Explicitely instantiate the desired classes
// (sync with typedefs at the end of the hpp file)
template class PinholeProjection<NoDistortion>;
template class PinholeProjection<RadialTangentialDistortion>;


} // namespace ze
