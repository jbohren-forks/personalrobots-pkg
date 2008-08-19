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

#ifndef DC1394_CAM_H
#define DC1394_CAM_H

#include <stdexcept>
#include "dc1394/dc1394.h"
#include <vector>

#include <opencv/cv.h> 

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

namespace dc1394_cam
{

  class CamException : public std::runtime_error
  {
  public:
    CamException(const char* msg) : std::runtime_error(msg) {}
  };

  class Cam;

  enum FrameOwnership {CAM_OWNS_BOTH, FRAME_OWNS_FRAME, FRAME_OWNS_BOTH, FRAME_OWNS_NONE};

  class FrameWrapper
  {
    //    friend class Cam;
    //    friend FrameWrapper debayerFrame(FrameWrapper&, dc1394color_filter_t, dc1394bayer_method_t);
    //    friend FrameWrapper undistortFrame(FrameWrapper, double, double, double, double, double, double, double, double);
    //  protected:
  public:
    FrameWrapper() : name_(std::string("")), frame_(NULL), parent_(NULL), ownership_(FRAME_OWNS_NONE) {}

    FrameWrapper(std::string name, dc1394video_frame_t* frame, Cam* parent, FrameOwnership ownership)
      : name_(name), frame_(frame), parent_(parent), ownership_(ownership) {}

  public:
    dc1394video_frame_t* getFrame() {return frame_;}

    std::string getName() {return name_;}
    
    Cam* getParent() {return parent_;};

    void releaseFrame();

  private:
    std::string name_;
    dc1394video_frame_t* frame_;
    Cam* parent_;
    FrameOwnership ownership_;
  };

  typedef std::vector<FrameWrapper> FrameSet;

  void init();
  void fini();
  size_t numCams();
  uint64_t getGuid(size_t i);
  bool waitForData(int usec);

  FrameWrapper debayerFrame(FrameWrapper& f, dc1394color_filter_t bayer, dc1394bayer_method_t method = DC1394_BAYER_METHOD_BILINEAR);

  void initUndistortFrame(FrameWrapper& fw, CvMat *intrinsic, CvMat *distortion, IplImage **mapx, IplImage **mapy);

  FrameWrapper undistortFrame(FrameWrapper& f, IplImage *mapx, IplImage *mapy);

  FrameWrapper undistortFrame(FrameWrapper& f, CvMat*, CvMat*);

  FrameWrapper undistortFrame(FrameWrapper& f, double fx, double fy,
                              double cx, double cy, double k1, double k2, double p1, double p2);

  class Cam
  {
    friend void init();
    friend void fini();
    friend size_t numCams();
    friend uint64_t getGuid(size_t i);
    friend bool waitForData(int usec);
    friend dc1394video_frame_t* debayerFrame(dc1394video_frame_t*, dc1394color_filter_t, dc1394bayer_method_t);

  protected:
    static dc1394_t* dcRef;
    static fd_set camFds;

    virtual void cleanup();

  public:

    Cam(uint64_t guid,
        dc1394speed_t speed = DC1394_ISO_SPEED_400,
        dc1394video_mode_t video = DC1394_VIDEO_MODE_640x480_MONO8,
        dc1394framerate_t fps = DC1394_FRAMERATE_30,
        size_t bufferSize = 8);

    virtual ~Cam();

    virtual void start();

    virtual void stop();

    virtual void enableColorization(dc1394color_filter_t bayer = DC1394_COLOR_FILTER_RGGB)
    {
      colorize_ = true;
      bayer_ = bayer;
    }

    virtual void disableColorization()
    {
      colorize_ = false;;
    }

    virtual void enableRectification(double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2)
    {
      rectify_ = true;
      init_rectify_ = true;
  
      CV_MAT_ELEM(*intrinsic_, float, 0, 0) = fx;
      CV_MAT_ELEM(*intrinsic_, float, 0, 2) = cx;
      CV_MAT_ELEM(*intrinsic_, float, 1, 1) = fy;
      CV_MAT_ELEM(*intrinsic_, float, 1, 2) = cy;
      CV_MAT_ELEM(*intrinsic_, float, 2, 2) = 1;
  
      CV_MAT_ELEM(*distortion_, float, 0, 0) = k1;
      CV_MAT_ELEM(*distortion_, float, 1, 0) = k2;
      CV_MAT_ELEM(*distortion_, float, 2, 0) = p1;
      CV_MAT_ELEM(*distortion_, float, 3, 0) = p2;
      
    }

    virtual void disableRectification()
    {
      rectify_ = false;
    }
    
    virtual FrameSet getFrames(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    virtual dc1394video_frame_t* getDc1394Frame(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    virtual void releaseDc1394Frame(dc1394video_frame_t* f);

    virtual void setFeature(dc1394feature_t feature, uint32_t value);

    virtual void setFeatureAbsolute(dc1394feature_t feature, float value);

    virtual void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    virtual void setControlRegister(uint64_t offset, uint32_t value);
    virtual uint32_t getControlRegister(uint64_t offset);

    bool started;
    dc1394camera_t* dcCam;

  protected:

    bool colorize_;
    bool rectify_;
    bool init_rectify_;
    dc1394color_filter_t bayer_;

    CvMat *intrinsic_;
    CvMat *distortion_;

    IplImage* mapx_;
    IplImage* mapy_;

  };

};

#endif
