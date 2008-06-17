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
  uint32_t options;

public:
  static const uint32_t CORRECT_BGR     = 0x0001;
  static const uint32_t MAXDEPTH_8U     = 0x0002;

  CvBridge(T *msg, uint32_t _options = 0) :
    ImageCodec<T>(msg),
    options(_options)  { }

  bool from_cv(IplImage *cv_image, int compression_quality = 90)
  {
    this->msg->width = cv_image->width;
    this->msg->height = cv_image->height;
    // TODO: verify colorspaces other than rgb and monochrome
    // return false for unhandled types
    this->msg->colorspace = "rgb24";
    this->realloc_raster_if_needed();
    if (cv_image->nChannels == 1)
    {
      for (int y = 0; y < cv_image->height; y++)
        for (int x = 0; x < cv_image->width; x++)
        {
          uint8_t *p = (uint8_t *)(cv_image->imageData + y * cv_image->widthStep + x);
          uint8_t *q = this->raster + y * cv_image->width * 3 + x * 3;
          (*q   )  = *p;
          (*(q+1)) = *p;
          (*(q+2)) = *p;
        }
    }
    else
      memcpy(this->raster, cv_image->imageData, this->get_raster_size());
    this->deflate(compression_quality);
    return true;
  }

  bool to_cv(IplImage **cv_image)
  {
    //    this->inflate("rgb24");
    this->inflate();
    if (!this->raster)
      return false;

    int channels = 1;
    int depth = IPL_DEPTH_8U;
    int elem_size = 1;
    bool needs_swap_rb = false;
    bool needs_depth_reduced = false;

    if (this->msg->colorspace == "rgb24") {
      channels = 3;
      elem_size = 1;
      depth = IPL_DEPTH_8U;
      needs_swap_rb = true;
    } else if (this->msg->colorspace == "mono8") {
      channels = 1;
      elem_size = 1;
      depth = IPL_DEPTH_8U;
    } else if (this->msg->colorspace == "mono16") {
      channels = 1;
      elem_size = 2;
      depth = IPL_DEPTH_16U;
      needs_depth_reduced = true;
    }

    *cv_image = cvCreateImage(cvSize(this->msg->width, this->msg->height),
                              depth, channels);

    // todo: ensure row alignment is ok (copy in one scanline at a time)                                                      
    for (size_t row = 0; row < this->msg->height; row++)
      memcpy((*cv_image)->imageData + row * (*cv_image)->widthStep,
             this->raster + row * this->msg->width * channels * elem_size, this->msg->width*channels * elem_size);

    if ((options & CORRECT_BGR) && needs_swap_rb)
      cvCvtColor(*cv_image, *cv_image, CV_RGB2BGR);

    if ((options & MAXDEPTH_8U) && needs_depth_reduced)
    {
      IplImage* cv_image_sc = cvCreateImage(cvSize(this->msg->width, this->msg->height),
                                            IPL_DEPTH_8U, channels);
      cvCvtScale(*cv_image, cv_image_sc, 0.0625, 0);
      cvReleaseImage(cv_image);
      *cv_image = cv_image_sc;
    }


    return true;
  }
};

#endif

