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
  

  void init();
  void fini();
  size_t numCams();
  uint64_t getGuid(size_t i);
  bool waitForData(int usec);

  class Cam
  {
  private:
    static dc1394_t* dcRef;
    static fd_set camFds;

    void cleanup();
    bool started;

  public:

    static void init();

    static void fini();

    static size_t numCams();

    static uint64_t getGuid(size_t i);

    static bool waitForData(int usec);

    Cam(uint64_t guid,
        dc1394speed_t speed = DC1394_ISO_SPEED_400,
        dc1394video_mode_t video = DC1394_VIDEO_MODE_640x480_MONO8,
        dc1394framerate_t fps = DC1394_FRAMERATE_30,
        size_t bufferSize = 8);

    ~Cam();

    void start();

    void stop();
    
    dc1394video_frame_t* getFrame(dc1394capture_policy_t policy = DC1394_CAPTURE_POLICY_WAIT);

    void releaseFrame(dc1394video_frame_t* f);

    void setFeature(dc1394feature_t feature, uint32_t value);

    void setFeatureAbsolute(dc1394feature_t feature, float value);

    void setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode);

    void setControlRegister(uint64_t offset, uint32_t value);
    uint32_t getControlRegister(uint64_t offset);

    dc1394camera_t* dcCam;


  };

};

#endif

