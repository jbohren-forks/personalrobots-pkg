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

#include "color_calib.h"

#include "opencv/cxcore.h"
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace color_calib;

color_calib::Calibration::Calibration(ros::node* node) : node_(node), color_cal_(NULL)
{
  color_cal_ = cvCreateMat(3, 3, CV_32FC1);
  color_cal_bgr_ = cvCreateMat(3, 3, CV_32FC1);
  cvSetIdentity(color_cal_, cvScalar(1.0));
  cvSetIdentity(color_cal_bgr_, cvScalar(1.0));
}

color_calib::Calibration::~Calibration()
{
  if (color_cal_)
    cvReleaseMat(&color_cal_);
  if (color_cal_bgr_)
    cvReleaseMat(&color_cal_bgr_);
}

bool
color_calib::Calibration::getFromParam(std::string topic_name)
{
  if (node_)
  {
    std::string color_cal_str = (node_->map_name(topic_name) + std::string("/color_cal"));

    if (node_->has_param(color_cal_str))
    {
      XmlRpc::XmlRpcValue xml_color_cal;
      node_->get_param(color_cal_str, xml_color_cal);
      for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
          cvmSet(color_cal_, i, j, (double)(xml_color_cal[3*i + j]));
        }
      populateBGR();
      return true;
    }
  }
  return false;
}

bool
color_calib::Calibration::setParam(std::string topic_name)
{
  if (node_)
  {
    std::string color_cal_str = (node_->map_name(topic_name) + std::string("/color_cal"));

    XmlRpc::XmlRpcValue xml_color_cal;
    for (int i = 2; i >= 0; i--)
      for (int j = 0; j < 3; j++)
        xml_color_cal[3*i + j] = cvmGet(color_cal_, i, j);
    
    node_->set_param(color_cal_str, xml_color_cal);

    return true;
  }
  else
  {
    return false;
  }
}


CvMat* 
color_calib::Calibration::getCal(uint32_t flag)
{
  if (flag & COLOR_CAL_BGR)
    return color_cal_bgr_;
  else 
    return color_cal_;
}

bool 
color_calib::Calibration::setCal(CvMat* cal, uint32_t flag)
{
  if (cal->height == 3 && cal->width==3)
  {
    if (flag & COLOR_CAL_BGR)
    {
      cvCopy(cal, color_cal_bgr_);
      populateRGB();
    } else {
      cvCopy(cal, color_cal_);
      populateBGR();
    }
    return true;
  }
  return false;
}

void 
color_calib::Calibration::populateBGR()
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvmSet(color_cal_bgr_, 2-i, 2-j, cvmGet(color_cal_, i, j));
}

void 
color_calib::Calibration::populateRGB()
{
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      cvmSet(color_cal_, 2-i, 2-j, cvmGet(color_cal_bgr_, i, j));
}


void
color_calib::Calibration::correctColor(IplImage* src, IplImage* dst, bool do_decompand, bool do_recompand, uint32_t flags)
{
  if (do_decompand)
    decompand(src, dst);
    
  if (flags & COLOR_CAL_BGR)
    cvTransform(dst, dst, color_cal_bgr_);
  else
    cvTransform(dst, dst, color_cal_);
    
  if (do_recompand)
    compand(dst, dst);
}
