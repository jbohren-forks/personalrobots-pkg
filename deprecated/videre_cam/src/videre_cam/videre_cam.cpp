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

#include <errno.h>
#include "videre_cam/videre_cam.h"

#include <sstream>

#include <iostream>


#define CHECK_READY() \
  if (!dcRef) { \
    char msg[256]; \
    snprintf(msg, 256, "Tried to call %s before calling dc1394_cam::Cam::init()", __FUNCTION__); \
    throw CamException(msg); \
  }

#define CHECK_ERR(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw CamException(msg); \
  } \
  }

#define CHECK_ERR_CLEAN(fnc, amsg) \
  { \
  dc1394error_t err = fnc; \
  if (err != DC1394_SUCCESS) { \
    cleanup(); \
    char msg[256]; \
    snprintf(msg, 256, "%s: %s", dc1394_error_get_string(err), amsg);        \
    throw CamException(msg); \
  }\
  }


template <class T>
void extract(std::string& data, std::string section, std::string param, T& t)
{
  std::istringstream iss(data.substr( data.find(param, data.find(section) ) + param.length()));
  iss >> t;
}

template <>
void extract<NEWMAT::Matrix>(std::string& data, std::string section, std::string param, NEWMAT::Matrix& m)
{
  std::istringstream iss(data.substr( data.find(param, data.find(section) ) + param.length()));

  for (int i = 1; i <= m.Nrows(); i++)
    for (int j = 1; j <= m.Ncols(); j++)
      iss >> m(i,j);
}


using namespace dc1394_cam;

videre_cam::VidereCam::VidereCam(uint64_t guid, 
                                 VidereMode proc_mode,
                                 dc1394speed_t speed,
                                 dc1394framerate_t fps,
                                 size_t bufferSize)
  : Cam(guid, speed, DC1394_VIDEO_MODE_640x480_YUV422, fps, bufferSize), proc_mode_(proc_mode), lproj_(3,4), rproj_(3,4), lrect_(3,3), rrect_(3,3)
{

  if (dcCam->vendor_id != 0x5505)
  {
    cleanup();
    throw VidereException("Not a Videre camera\n");
  }

  // Read in param file and parse where necessary.

  // Extract the Videre calibration file:
  // check for calibration
  uint32_t qval = getControlRegister(0xF0800);
  if (qval == 0xffffffff)
  {
    cleanup();
    throw VidereException("Videre is missing calibration file\n");
  }

  char buf[4096*4];
  int n = 4096*4;
  char* bb = buf;

  // read in each byte
  int pos = 0;
  uint32_t quad;
  quad = getControlRegister(0xF0800+pos);

  while (quad != 0x0 && quad != 0xffffffff && n > 3)
  {
    int val;
    pos += 4;
    n -= 4;
    val = (quad >> 24) & 0xff;
    *bb++ = val;
    val = (quad >> 16) & 0xff;
    *bb++ = val;
    val = (quad >> 8) & 0xff;
    *bb++ = val;
    val = quad & 0xff;
    *bb++ = val;
    quad = getControlRegister(0xF0800+pos);
  }
  *bb = 0; // just in case we missed the last zero

  cal_params_ = buf;

  extract(cal_params_, "[left camera]", "proj", lproj_);
  extract(cal_params_, "[left camera]", "rect", lrect_);

  extract(cal_params_, "[left camera]", "Cx",     lCx_);
  extract(cal_params_, "[left camera]", "Cy",     lCy_);
  extract(cal_params_, "[left camera]", "f ",      lf_);
  extract(cal_params_, "[left camera]", "fy",     lfy_);
  extract(cal_params_, "[left camera]", "kappa1", lk1_);
  extract(cal_params_, "[left camera]", "kappa2", lk2_);
  extract(cal_params_, "[left camera]", "kappa3", lk3_);
  extract(cal_params_, "[left camera]", "tau1",   lt1_);
  extract(cal_params_, "[left camera]", "tau2",   lt2_);

  extract(cal_params_, "[right camera]", "proj", rproj_);
  extract(cal_params_, "[right camera]", "rect", rrect_);

  extract(cal_params_, "[right camera]", "Cx",     rCx_);
  extract(cal_params_, "[right camera]", "Cy",     rCy_);
  extract(cal_params_, "[right camera]", "f ",      rf_);
  extract(cal_params_, "[right camera]", "fy",     rfy_);
  extract(cal_params_, "[right camera]", "kappa1", rk1_);
  extract(cal_params_, "[right camera]", "kappa2", rk2_);
  extract(cal_params_, "[right camera]", "kappa3", rk3_);
  extract(cal_params_, "[right camera]", "tau1",   rt1_);
  extract(cal_params_, "[right camera]", "tau2",   rt2_);

  extract(cal_params_, "[image]", "width",      w_);
  extract(cal_params_, "[image]", "height",     h_);
  extract(cal_params_, "[stereo]", "corrxsize", corrs_);
  
  logs_ = 9;
  
  extract(cal_params_, "[stereo]", "offx", offx_);
  
  dleft_   = (logs_ + corrs_ - 2)/2 - 1 + offx_;
  dwidth_  = w_ - (logs_ + corrs_ + offx_ - 2);
  dtop_    = (logs_ + corrs_ - 2)/2;
  dheight_ = h_ - (logs_ + corrs_);


  init_rectify_ = true;

  intrinsic_ = cvCreateMat(3,3,CV_32FC1);
  distortion_ = cvCreateMat(5,1,CV_32FC1);
  rectification_ = cvCreateMat(3,3,CV_32FC1);
  rectified_intrinsic_ = cvCreateMat(3,3,CV_32FC1);

  r_intrinsic_ = cvCreateMat(3,3,CV_32FC1);
  r_distortion_ = cvCreateMat(5,1,CV_32FC1);
  r_rectification_ = cvCreateMat(3,3,CV_32FC1);
  r_rectified_intrinsic_ = cvCreateMat(3,3,CV_32FC1);

  r_mapx_ = NULL;
  r_mapy_ = NULL;

  color_capable_ = (getControlRegister(0x50c) & 0x80000000UL);

  bayer_ = DC1394_COLOR_FILTER_GRBG;

  //  std::cout << "Read in params: " << std::endl << cal_params_;
}



void
videre_cam::VidereCam::start()
{
  dc1394_cam::Cam::start();

  usleep(100000);

  uint32_t qval1 = 0x08000000 | (0x90 << 16) | ( ( proc_mode_ & 0x7) << 16);
  uint32_t qval2 = 0x08000000 | (0x9C << 16);
  
  setControlRegister(0xFF000, qval1);
  setControlRegister(0xFF000, qval2);
  
  usleep(250000);
  setControlRegister(0xFF000, qval1);
  setControlRegister(0xFF000, qval2);
  
  usleep(250000);
  setControlRegister(0xFF000, qval1);
  setControlRegister(0xFF000, qval2);
}


void
videre_cam::VidereCam::setTextureThresh(int thresh)
{
        usleep(50000);

        if (thresh < 0)
          thresh = 0;
        if (thresh > 63)
          thresh = 63;
        
        uint32_t t_thresh = 0x08000000 | (0x40 << 16) | ( thresh << 16);
        setControlRegister(0xFF000, t_thresh);
}

void
videre_cam::VidereCam::setUniqueThresh(int thresh)
{
        usleep(50000);

        if (thresh < 0)
          thresh = 0;
        if (thresh > 63)
          thresh = 63;

        uint32_t u_thresh = 0x08000000 | (0x00 << 16) | ( thresh << 16);
        setControlRegister(0xFF000, u_thresh);
}

void
videre_cam::VidereCam::setCompanding(bool companding)
{
        usleep(50000);

        if (companding)
          setControlRegister(0xFF000, 0x041C0003);
        else
          setControlRegister(0xFF000, 0x041C0002);
}

void
videre_cam::VidereCam::setHDR(bool hdr)
{
        usleep(50000);

        if (hdr)
          setControlRegister(0xFF000, 0x040F0051);
        else
          setControlRegister(0xFF000, 0x040F0011);
  
}





// This is what dc1394_cam does, except we don't do anything that requires reading from a control register
void
videre_cam::VidereCam::setFeature(dc1394feature_t feature, uint32_t value, uint32_t value2)
{
  CHECK_READY();
  if (feature == DC1394_FEATURE_WHITE_BALANCE)
  {
    CHECK_ERR_CLEAN( dc1394_feature_whitebalance_set_value_blind(dcCam, value, value2), "Could not set feature");
  }
  else
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_value_blind(dcCam, feature, value), "Could not set feature");
  }
}

void
videre_cam::VidereCam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_feature_set_mode_blind(dcCam, feature, mode), "Could not set feature");
}


void
videre_cam::VidereCam::enableRectification(double fx, double fy, double cx, double cy, double k1, double k2, double k3, double p1, double p2)
{
  Cam::enableRectification(fx, fy, cx, cy, k1, k2, k3, p1, p2);

  CV_MAT_ELEM(*r_intrinsic_, float, 0, 0) = fx;
  CV_MAT_ELEM(*r_intrinsic_, float, 0, 2) = cx;
  CV_MAT_ELEM(*r_intrinsic_, float, 1, 1) = fy;
  CV_MAT_ELEM(*r_intrinsic_, float, 1, 2) = cy;
  CV_MAT_ELEM(*r_intrinsic_, float, 2, 2) = 1;
  
  CV_MAT_ELEM(*r_distortion_, float, 0, 0) = k1;
  CV_MAT_ELEM(*r_distortion_, float, 1, 0) = k2;
  CV_MAT_ELEM(*r_distortion_, float, 2, 0) = p1;
  CV_MAT_ELEM(*r_distortion_, float, 3, 0) = p2;
  CV_MAT_ELEM(*r_distortion_, float, 4, 0) = k3;

  CV_MAT_ELEM(*r_rectification_, float, 0, 0) = 1.0;
  CV_MAT_ELEM(*r_rectification_, float, 1, 1) = 1.0;
  CV_MAT_ELEM(*r_rectification_, float, 2, 2) = 1.0;

  CV_MAT_ELEM(*r_rectified_intrinsic_, float, 0, 0) = fx;
  CV_MAT_ELEM(*r_rectified_intrinsic_, float, 0, 2) = cx;
  CV_MAT_ELEM(*r_rectified_intrinsic_, float, 1, 1) = fy;
  CV_MAT_ELEM(*r_rectified_intrinsic_, float, 1, 2) = cy;
  CV_MAT_ELEM(*r_rectified_intrinsic_, float, 2, 2) = 1;

}


void
videre_cam::VidereCam::enableRectification()
{
  rectify_ = true; 
  init_rectify_ = true;

  CV_MAT_ELEM(*intrinsic_, float, 0, 0) = lf_;
  CV_MAT_ELEM(*intrinsic_, float, 0, 2) = lCx_;
  CV_MAT_ELEM(*intrinsic_, float, 1, 1) = lfy_;
  CV_MAT_ELEM(*intrinsic_, float, 1, 2) = lCy_;
  CV_MAT_ELEM(*intrinsic_, float, 2, 2) = 1;
    
  CV_MAT_ELEM(*distortion_, float, 0, 0) = lk1_;
  CV_MAT_ELEM(*distortion_, float, 1, 0) = lk2_;
  CV_MAT_ELEM(*distortion_, float, 2, 0) = lt1_;
  CV_MAT_ELEM(*distortion_, float, 3, 0) = lt2_;
  CV_MAT_ELEM(*distortion_, float, 4, 0) = lk3_;
    
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      CV_MAT_ELEM(*rectification_, float, i, j) = lrect_(i+1,j+1);
      CV_MAT_ELEM(*rectified_intrinsic_, float, i, j) = lproj_(i+1,j+1);
    }

  CV_MAT_ELEM(*r_intrinsic_, float, 0, 0) = rf_;
  CV_MAT_ELEM(*r_intrinsic_, float, 0, 2) = rCx_;
  CV_MAT_ELEM(*r_intrinsic_, float, 1, 1) = rfy_;
  CV_MAT_ELEM(*r_intrinsic_, float, 1, 2) = rCy_;
  CV_MAT_ELEM(*r_intrinsic_, float, 2, 2) = 1;
    
  CV_MAT_ELEM(*r_distortion_, float, 0, 0) = rk1_;
  CV_MAT_ELEM(*r_distortion_, float, 1, 0) = rk2_;
  CV_MAT_ELEM(*r_distortion_, float, 2, 0) = rt1_;
  CV_MAT_ELEM(*r_distortion_, float, 3, 0) = rt2_;
  CV_MAT_ELEM(*r_distortion_, float, 4, 0) = rk3_;

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
      CV_MAT_ELEM(*r_rectification_, float, i, j) = rrect_(i+1,j+1);
      CV_MAT_ELEM(*r_rectified_intrinsic_, float, i, j) = rproj_(i+1,j+1);
    }
}



dc1394_cam::FrameSet
videre_cam::VidereCam::getFrames(dc1394capture_policy_t policy)
{
  FrameSet fs;

  dc1394video_frame_t* dcframe = getDc1394Frame(policy);

  if (dcframe != NULL)
  {
    dc1394video_frame_t* dcframe1 = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
    dc1394video_frame_t* dcframe2 = (dc1394video_frame_t*)calloc(1,sizeof(dc1394video_frame_t));
    
    dc1394_deinterlace_stereo_frames(dcframe, dcframe1, DC1394_STEREO_METHOD_INTERLACED);

    releaseDc1394Frame(dcframe);

    if (dcframe1 != NULL)
    {
      *dcframe2 = *dcframe1;
      dcframe1->size[1] = dcframe2->size[1] = dcframe1->size[1]/2;

      dcframe2->image = (unsigned char*)malloc(dcframe2->size[0]*dcframe2->size[1]);
      memcpy(dcframe2->image, dcframe1->image + dcframe1->size[0]*dcframe1->size[1], dcframe1->size[0]*dcframe1->size[1]);

      if (proc_mode_ == PROC_MODE_DISPARITY || proc_mode_ == PROC_MODE_DISPARITY_RAW)
      {
        /*
         * disparity image size
         * ====================
         * dleft  : (logs + corrs - 2)/2 - 1 + offx
         * dwidth : w - (logs + corrs + offx - 2)
         * dtop   : (logs + corrs - 2)/2
         * dheight: h - (logs + corrs)
         *
         * source rectangle
         * ================
         * (w-dwidth, h-dheight) => upper left
         * (dwidth, dheight)     => size
         *
         * dest rectangle
         * ==============
         * (dleft-6,dtop)        => upper left
         * (dwidth,dheight)      => size
         */
        for (int i = 0; i < dheight_; i++)
        {
          memcpy(dcframe1->image + (dtop_ + i)*w_ + dleft_ - 6, dcframe1->image + (h_ - dheight_ + i)*w_ + w_ - dwidth_, dwidth_);
          memset(dcframe1->image + (dtop_ + i)*w_ + dleft_ - 6 + dwidth_, 0, w_ - dwidth_ - dleft_ + 6);
        }
        for (int i = dheight_ + dtop_; i < h_; i++)
        {
          memset(dcframe1->image + i*w_, 0, w_);
        }
      }

      FrameWrapper fw1;
      FrameWrapper fw2;
        
      switch (proc_mode_)
      {
      case PROC_MODE_OFF:
      case PROC_MODE_NONE:
        fw1 = FrameWrapper("right", dcframe1, this, FRAME_OWNS_BOTH);
        fw2 = FrameWrapper("left",  dcframe2, this, FRAME_OWNS_BOTH);
        if (colorize_)
        {
          FrameWrapper fw1_color = dc1394_cam::debayerFrame(fw1, bayer_);
          FrameWrapper fw2_color = dc1394_cam::debayerFrame(fw2, bayer_);
          fw1.releaseFrame();
          fw2.releaseFrame();
          fw1 = fw1_color;
          fw2 = fw2_color;
        }

        if (rectify_)
        {
          if (init_rectify_)
          {
            initUndistortFrame(fw1, r_intrinsic_, r_distortion_, r_rectification_, r_rectified_intrinsic_, &r_mapx_, &r_mapy_);
            initUndistortFrame(fw2, intrinsic_, distortion_, rectification_, rectified_intrinsic_, &mapx_, &mapy_);
            init_rectify_ = false;
          }

          FrameWrapper fw1_rect = dc1394_cam::undistortFrame(fw1, r_mapx_, r_mapy_);
          FrameWrapper fw2_rect = dc1394_cam::undistortFrame(fw2, mapx_, mapy_);
          
          fw1.releaseFrame();
          fw2.releaseFrame();
          
          fw1 = fw1_rect;
          fw2 = fw2_rect;
        }

        break;
      case PROC_MODE_RECTIFIED:
        fw1 = FrameWrapper("right_rectified", dcframe1, this, FRAME_OWNS_BOTH);
        fw2 = FrameWrapper("left_rectified",  dcframe2, this, FRAME_OWNS_BOTH);
        break;
      case PROC_MODE_DISPARITY:
        fw1 = FrameWrapper("left_disparity", dcframe1, this, FRAME_OWNS_BOTH);
        fw2 = FrameWrapper("left_rectified",  dcframe2, this, FRAME_OWNS_BOTH);
        break;
      case PROC_MODE_DISPARITY_RAW:
        fw1 = FrameWrapper("left_disparity", dcframe1, this, FRAME_OWNS_BOTH);
        fw2 = FrameWrapper("left",  dcframe2, this, FRAME_OWNS_BOTH);

        if (colorize_)
        {
          FrameWrapper fw2_color = dc1394_cam::debayerFrame(fw2, bayer_);
          fw2.releaseFrame();
          fw2 = fw2_color;
        }
        
        if (rectify_)
        {
          if (init_rectify_)
          {
            initUndistortFrame(fw2, intrinsic_, distortion_, rectification_, rectified_intrinsic_, &mapx_, &mapy_);
            init_rectify_ = false;
          }
          
          FrameWrapper fw2_rect = dc1394_cam::undistortFrame(fw2, mapx_, mapy_);
          
          fw2.releaseFrame();
          
          fw2 = fw2_rect;
        }
        break;
      default:
        break;
      }

      fs.push_back(fw1);
      fs.push_back(fw2);
    }

  }
    return fs;
}
