// Copyright (C) ETH Zurich, Wyss Zurich, Zurich Eye - All Rights Reserved
// Unauthorized copying of this file, via any medium is strictly prohibited
// Proprietary and confidential

#include <ze/cameras/camera_impl.hpp>

namespace ze {

//-----------------------------------------------------------------------------
// Explicitely instantiate the desired classes.
// (sync with typedefs at the end of the hpp file)
template class PinholeProjection<NoDistortion>;
template class PinholeProjection<FovDistortion>;
template class PinholeProjection<RadialTangentialDistortion>;
template class PinholeProjection<EquidistantDistortion>;

} // namespace ze
