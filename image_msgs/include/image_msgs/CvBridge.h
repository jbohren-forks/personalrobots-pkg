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


namespace image_msgs
{

  class CvBridge
  {
    IplImage* img_;
    bool owns_data_;

  public:

    CvBridge() : img_(0), owns_data_(false)
    {
      img_ = cvCreateImageHeader( cvSize(0,0), IPL_DEPTH_8U, 1 );
    }

    ~CvBridge()
    {
      if (owns_data_)
        cvReleaseData(&img_);
      
      cvReleaseImageHeader(&img_);
    }


    inline IplImage* toIpl()
    {
      return img_;
    }

    bool fromImage(Image& rosimg)
    {
      if (rosimg.depth == "byte")
      {
        cvInitImageHeader(img_, cvSize(rosimg.byte_data.layout.dim[1].size, rosimg.byte_data.layout.dim[0].size), IPL_DEPTH_8U, 1);
        cvSetData(img_, &(rosimg.byte_data.data[0]), rosimg.byte_data.layout.dim[1].stride);
      } else if (rosimg.depth == "uint16") {
        cvInitImageHeader(img_, cvSize(rosimg.uint16_data.layout.dim[1].size, rosimg.uint16_data.layout.dim[0].size), IPL_DEPTH_16U, 1);
        cvSetData(img_, &(rosimg.uint16_data.data[0]), rosimg.uint16_data.layout.dim[1].stride*sizeof(uint16_t));
      }
      
      return true;
        
    }
  private:

  };
}


#endif
