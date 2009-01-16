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

#ifndef COLORCALIB_HH
#define COLORCALIB_HH

#include "opencv/cxcore.h"
#include <string>
#include "ros/node.h"

static const uint32_t COLOR_CAL_BGR = 1;
static const uint32_t COLOR_CAL_COMPAND_DISPLAY = 1 << 2;

namespace color_calib
{

  class Calibration
  {
    ros::Node* node_;

    CvMat* color_cal_;
    CvMat* color_cal_bgr_;
  public:
    Calibration(ros::Node* node);
    ~Calibration();

    bool getFromParam(std::string topic_name_);
    bool setParam(std::string topic_name_);

    CvMat* getCal(uint32_t flag = 0);
    bool setCal(CvMat* cal, uint32_t flag = 0);

    void correctColor(IplImage* src, IplImage* dst, bool do_decompand, bool do_recompand, uint32_t flags = 0);

  private:
    void populateBGR();
    void populateRGB();
  };

  float srgb2lrgb(float x);
  float lrgb2srgb(float x);
  void decompand(IplImage* src, IplImage* dst);
  void compand(IplImage* src, IplImage* dst);
  bool find_calib(IplImage* img,  Calibration& cal, uint32_t flags=0);
}

#endif
