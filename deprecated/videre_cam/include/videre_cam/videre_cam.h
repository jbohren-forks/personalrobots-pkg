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

#ifndef VIDERE_CAM_H
#define VIDERE_CAM_H

#include <stdexcept>
#include "dc1394_cam/dc1394_cam.h"
#include <newmat10/newmat.h>

namespace videre_cam
{

  class VidereException : public dc1394_cam::CamException
  {
  public:
    VidereException(const char* msg) : dc1394_cam::CamException(msg) {}
  };
  
  enum VidereMode { PROC_MODE_OFF, PROC_MODE_NONE, PROC_MODE_TEST, PROC_MODE_RECTIFIED, PROC_MODE_DISPARITY, PROC_MODE_DISPARITY_RAW };

  class VidereCam : public dc1394_cam::Cam
  {

  public:
    
    VidereCam(uint64_t guid,
              VidereMode proc_mode = PROC_MODE_NONE,
              dc1394speed_t speed = DC1394_ISO_SPEED_400,
              dc1394framerate_t fps = DC1394_FRAMERATE_30,
              size_t bufferSize = 8);

    virtual ~VidereCam()
    {
      cvReleaseMat(&r_intrinsic_);
      cvReleaseMat(&r_distortion_);
      cvReleaseMat(&r_rectification_);
      cvReleaseMat(&r_rectified_intrinsic_);
      
      cvReleaseImage(&r_mapx_);
      cvReleaseImage(&r_mapy_);
    };

    virtual void start();

    virtual dc1394_cam::FrameSet getFrames(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    virtual void enableColorization(dc1394color_filter_t bayer = DC1394_COLOR_FILTER_RGGB) { colorize_ = color_capable_; }

    virtual void enableRectification(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2);

    virtual void setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2 = 0);

    virtual void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    void enableRectification();

    VidereMode getMode() { return proc_mode_; }

    void setTextureThresh(int thresh);

    void setUniqueThresh(int thresh);

    void setCompanding(bool companding);

    void setHDR(bool hdr);

    std::string getCalParams() {return cal_params_;}

    NEWMAT::Matrix& getLProj() { return lproj_; }
    NEWMAT::Matrix& getRProj() { return rproj_; };
    NEWMAT::Matrix& getLRect() { return lrect_; };
    NEWMAT::Matrix& getRRect() { return rrect_; };

private:

    VidereMode proc_mode_;
    std::string cal_params_;

    bool color_capable_;

    NEWMAT::Matrix lproj_;
    NEWMAT::Matrix rproj_;
    NEWMAT::Matrix lrect_;
    NEWMAT::Matrix rrect_;

    float lCx_;
    float lCy_;
    float lf_;
    float lfy_;
    float lk1_;
    float lk2_;
    float lk3_;
    float lt1_;
    float lt2_;

    float rCx_;
    float rCy_;
    float rf_;
    float rfy_;
    float rk1_;
    float rk2_;
    float rk3_;
    float rt1_;
    float rt2_;

    int w_;
    int h_;
    int corrs_;
    int logs_;
    int offx_;
    int dleft_;
    int dwidth_;
    int dtop_;
    int dheight_;


    CvMat *r_intrinsic_;
    CvMat *r_distortion_;
    CvMat *r_rectification_;
    CvMat *r_rectified_intrinsic_;

    IplImage* r_mapx_;
    IplImage* r_mapy_;

  };

};

#endif

