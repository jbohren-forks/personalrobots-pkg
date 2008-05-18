///////////////////////////////////////////////////////////////////////////////
// This file provides a handy bridge between OpenCV and ROS Image flows
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#ifndef IMAGE_UTILS_CV_BRIDGE_H
#define IMAGE_UTILS_CV_BRIDGE_H

#include "image_utils/image_codec.h"
#include "opencv/cv.h"

template <class T>
class CvBridge : public ImageCodec<T>
{
public:
  CvBridge(T *msg) :
    ImageCodec<T>(msg)
  {
  }

  bool from_cv(IplImage *cv_image, int compression_quality = 90)
  {
    this->msg->width = cv_image->width;
    this->msg->height = cv_image->height;
    // TODO: verify colorspace, handle monochrome, etc.
    // return false for unhandled types
    this->msg->colorspace = "rgb24";
    this->realloc_raster_if_needed();
    memcpy(this->raster, cv_image->imageData, this->get_raster_size());
    this->deflate(compression_quality);
    return true;
  }

  bool to_cv(IplImage **cv_image)
  {
    this->inflate("rgb24");
    if (!this->raster)
      return false;
    *cv_image = cvCreateImage(cvSize(this->msg->width, this->msg->height), 
                              IPL_DEPTH_8U, 3);
    // todo: ensure row alignment is ok (copy in one scanline at a time)
    for (size_t row = 0; row < this->msg->height; row++)
      memcpy((*cv_image)->imageData + row * (*cv_image)->widthStep, 
             this->raster + row * this->msg->width * 3, this->msg->width*3);
    return true;
  }
};

#endif

