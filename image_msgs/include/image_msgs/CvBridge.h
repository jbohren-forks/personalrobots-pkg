/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef CVBRIDGE_HH
#define CVBRIDGE_HH

#include "image_msgs/Image.h"
#include "opencv/cxcore.h"
#include "opencv/cv.h"

namespace image_msgs
{

  class CvBridge
  {
    IplImage* img_;
    IplImage* rosimg_;
    IplImage* cvtimg_;

    void reallocIfNeeded_(IplImage** img, CvSize sz, int depth, int channels)
    {
      if ((*img) != 0)
        if ((*img)->width     != sz.width  || 
            (*img)->height    != sz.height || 
            (*img)->depth     != depth     ||
            (*img)->nChannels != channels)
        {
          cvReleaseImage(img);
          *img = 0;
        }

      if (*img == 0)
      {
        *img = cvCreateImage(sz, depth, channels);
      }      
    }

  public:

    CvBridge() : img_(0), rosimg_(0), cvtimg_(0)
    {
      rosimg_ = cvCreateImageHeader( cvSize(0,0), IPL_DEPTH_8U, 1 );
    }

    ~CvBridge()
    {
      if (rosimg_)
        cvReleaseImageHeader(&rosimg_);
      
      if (cvtimg_)
        cvReleaseImage(&cvtimg_);
    }


    inline IplImage* toIpl()
    {
      return img_;
    }

    bool fromImage(Image& rosimg, std::string encoding = "")
    {
      if (rosimg.depth == "byte")
      {
        cvInitImageHeader(rosimg_, cvSize(rosimg.byte_data.layout.dim[1].size, rosimg.byte_data.layout.dim[0].size),
                          IPL_DEPTH_8U, rosimg.byte_data.layout.dim[2].size);
        cvSetData(rosimg_, &(rosimg.byte_data.data[0]), rosimg.byte_data.layout.dim[1].stride);
        img_ = rosimg_;
      } else if (rosimg.depth == "uint16") {
        cvInitImageHeader(rosimg_, cvSize(rosimg.uint16_data.layout.dim[1].size, rosimg.uint16_data.layout.dim[0].size),
                          IPL_DEPTH_16U, rosimg.uint16_data.layout.dim[2].size);
        cvSetData(rosimg_, &(rosimg.uint16_data.data[0]), rosimg.uint16_data.layout.dim[1].stride*sizeof(uint16_t));
        img_ = rosimg_;
      } else {
        return false;
      }

      if (encoding != "" && (encoding != rosimg.encoding))
      {
        if (encoding == "bgr" && rosimg.encoding == "rgb")
        {
          reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_RGB2BGR);
          img_ = cvtimg_;
        }
        else if (encoding == "bgr" && rosimg.encoding == "mono" )
        {
          reallocIfNeeded(&cvtimg_, IPL_DEPTH_8U, 3);
          cvCvtColor(rosimg_, cvtimg_, CV_GRAY2BGR);
          img_ = cvtimg_;
        }
        else
        {
          return false;
        }
      }
      return true;
    }

    void reallocIfNeeded(IplImage** img, int depth = -1, int channels = -1)
    {
      if (depth == -1)
        depth = img_->depth;
      if (channels == -1)
        channels = img_->nChannels;
      reallocIfNeeded_(img, cvGetSize(img_), depth, channels);
    }
  };
}


#endif
