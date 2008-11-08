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

#ifndef IMAGEWRAPPER_HH
#define IMAGEWRAPPER_HH

#include "image_msgs/Image.h"
#include "image.h"
#include "opencv/cxcore.h"


namespace image_msgs
{

  class ImageWrapper : public image_msgs::Image
  {
  public:
    bool fromInterlacedData(std::string label_arg,
                            uint32_t height_arg, uint32_t width_arg, uint32_t channel_arg,
                            std::string encoding_arg, std::string depth_arg,
                            void* data_arg,
                            uint32_t channel_step = 0, uint32_t width_step = 0, uint32_t height_step = 0)
    {
      label    = label_arg;
      encoding = encoding_arg;
      depth    = depth_arg;

      if (channel_step == 0)
        channel_step = channel_arg;

      if (width_step == 0)
        width_step = width_arg * channel_step;

      if (height_step == 0)
        height_step = height_arg * width_step;

      if (depth == "byte")
        fromDataHelper(byte_data,
                       height_arg, height_step,
                       width_arg, width_step,
                       channel_arg, channel_step,
                       data_arg);

      else if (depth == "uint16")
        fromDataHelper(uint16_data,
                       height_arg, height_step,
                       width_arg, width_step,
                       channel_arg, channel_step,
                       data_arg);

      return true;
    }

    IplImage* asIplImage()
    {
      IplImage* img;
      if (depth == "byte")
      {
        img = cvCreateImageHeader(cvSize(byte_data.layout.dim[1].size, byte_data.layout.dim[0].size), IPL_DEPTH_8U, 1);
        cvSetData(img, &(byte_data.data[0]), byte_data.layout.dim[1].stride);
      } else if (depth == "uint16") {
        img = cvCreateImageHeader(cvSize(uint16_data.layout.dim[1].size, uint16_data.layout.dim[0].size), IPL_DEPTH_16U, 1);
        cvSetData(img, &(uint16_data.data[0]), uint16_data.layout.dim[1].stride*sizeof(uint16_t));
      }
      return img;
    }
  private:
    template <class M>
    void fromDataHelper(M &m,
                           uint32_t sz0, uint32_t st0,
                           uint32_t sz1, uint32_t st1,
                           uint32_t sz2, uint32_t st2,
                           void *d)
    {
      m.layout.dim.resize(3);
      m.layout.dim[0].label  = "height";
      m.layout.dim[0].size   = sz0;
      m.layout.dim[0].stride = st0;
      m.layout.dim[1].label  = "width";
      m.layout.dim[1].size   = sz1;
      m.layout.dim[1].stride = st1;
      m.layout.dim[2].label  = "channel";
      m.layout.dim[2].size   = sz2;
      m.layout.dim[2].stride = st2;
      m.data.resize(st0);
      memcpy((char*)(&m.data[0]), (char*)(d), st0*sizeof(m.data[0]));
    }
  };
}


#endif
