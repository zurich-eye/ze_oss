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
#pragma once

#include <memory>
#include <glog/logging.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <imp/bridge/opencv/image_cv.hpp>
#include <ze/common/file_utils.hpp>

namespace ze {

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeLoad(ImageCvPtr<Pixel>& out,
                  const std::string& filename, PixelOrder pixel_order)
{
  CHECK(fileExists(filename)) << "File does not exist: " << filename;
  cv::Mat mat;
  if (pixel_order == PixelOrder::gray)
  {
    mat = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
    CHECK(!mat.empty());
  }
  else
  {
    // everything else needs color information :)
    mat = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    CHECK(!mat.empty());
  }

  switch(pixel_type<Pixel>::type)
  {
  case PixelType::i8uC1:
    if (pixel_order == PixelOrder::gray)
    {
      out = std::make_shared<ImageCv<Pixel>>(mat);
    }
    else
    {
      out = std::make_shared<ImageCv<Pixel>>(mat.cols, mat.rows);
      cv::cvtColor(mat, out->cvMat(), CV_BGR2GRAY);
    }
  break;
  case PixelType::i32fC1:
    out = std::make_shared<ImageCv<Pixel>>(mat.cols, mat.rows);
    if (mat.channels() > 1)
    {
      cv::cvtColor(mat, mat, CV_BGR2GRAY);
    }
    mat.convertTo(out->cvMat(), CV_32F, 1./255.);
  break;
  default:
    CHECK(false) << "Conversion for reading given pixel_type not supported yet.";
    break;
  }
}

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeSave(const std::string& filename, const ImageCv<Pixel>& img, bool normalize=false)
{
  if (normalize)
  {
    // TODO
  }
  cv::imwrite(filename, img.cvMat());
}

//------------------------------------------------------------------------------
template<typename Pixel>
void cvBridgeShow(const std::string& winname, const ImageCv<Pixel>& img,
                  bool normalize=false)
{
  if (normalize)
  {
    int mat_type = (img.nChannels() > 1) ? CV_8UC3 : CV_8UC1;
    cv::Mat norm_mat(img.height(), img.width(), mat_type);
    cv::normalize(img.cvMat(), norm_mat, 0, 255, CV_MINMAX, CV_8U);
    cv::imshow(winname, norm_mat);
  }
  else
  {
    cv::imshow(winname, img.cvMat());
  }
}

//------------------------------------------------------------------------------
template<typename Pixel, typename T>
void cvBridgeShow(const std::string& winname, const ImageCv<Pixel>& img,
                  const T& min, const T& max)
{
  cv::Mat norm_mat = img.cvMat().clone();
  norm_mat = (norm_mat-min) / (max-min);
  cv::imshow(winname, norm_mat);
}

//-----------------------------------------------------------------------------
// Convenience functions.
ImageCv8uC1::Ptr cvBridgeLoad8uC1(const std::string& filename);

//------------------------------------------------------------------------------
//enum class OcvBridgeLoadAs
//{
//  raw,
//  cuda,
//  cvmat
//};
//
//template<typename Pixel>
//std::shared_ptr<> ocv_bridge_imread(const std::string& filename, OcvBridgeLoadAs load_as=OcvBridgeLoadAs::raw)
//{
//  switch (load_as)
//  {
//  case OcvBridgeLoadAs::cvmat:
//    break;
//  case OcvBridgeLoadAs::cuda:
//    break;
//  case OcvBridgeLoadAs::raw:
//  default:
//    break;

//  }
//}


} // namespace ze
