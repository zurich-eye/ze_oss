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
#include <imp/bridge/af/orb_detector_af.hpp>

namespace ze {

OrbDetectorAF::OrbDetectorAF(const OrbDetectorOptions& options, const Size2u& image_size)
  : AbstractDetector(image_size, DetectorType::Orb),
    options_(options)
{ }

uint32_t OrbDetectorAF::detect(const ImagePyramid8uC1& pyr, KeypointsWrapper& keypoints)
{
  LOG(FATAL) << "Not implemented";
  return 0;
}

uint32_t OrbDetectorAF::detect(const ImageAF32fC1& im, OrbKeypointWrapper& keypoints)
{
  af::features feat;
  af::array desc;
  af::orb(
        feat,
        desc,
        im.afArray(),
        options_.fast_threshold,
        options_.max_num_features,
        options_.scale_factor,
        options_.pyramid_levels,
        options_.blur_input_image);

  const size_t num_detected = feat.getNumFeatures();
  keypoints.allocate(num_detected);
  feat.getX().host(keypoints.x.get());
  feat.getY().host(keypoints.y.get());
  feat.getScore().host(keypoints.score.get());
  feat.getOrientation().host(keypoints.orient.get());
  feat.getSize().host(keypoints.size.get());
  desc.host(keypoints.descr.get());

  return num_detected;
}

} // ze namespace
