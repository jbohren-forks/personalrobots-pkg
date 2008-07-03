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

#include "dc1394_cam/dc1394_cam.h"
#include <errno.h>

dc1394_t* dc1394_cam::Cam::dcRef = NULL;
fd_set dc1394_cam::Cam::camFds;


void
dc1394_cam::init() {
  Cam::init();
}

void
dc1394_cam::Cam::init() {
  if (dcRef == NULL)
  {
    dcRef = dc1394_new();
    FD_ZERO(&camFds);
  }
}



void
dc1394_cam::fini()
{
  Cam::fini();
}

void
dc1394_cam::Cam::fini()
{
  if (dcRef != NULL)
  {
    dc1394_free(dcRef);
  }
}



size_t dc1394_cam::numCams()
{
  return Cam::numCams();
}

size_t dc1394_cam::Cam::numCams() {
  CHECK_READY();

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcRef, &list), "Could not enumerate cameras" );

  size_t num = list->num;

  dc1394_camera_free_list (list);

  return num;
}


uint64_t dc1394_cam::getGuid(size_t i)
{
  return Cam::getGuid(i);
}

uint64_t dc1394_cam::Cam::getGuid(size_t i)
{
  CHECK_READY();

  dc1394camera_list_t * list;
  CHECK_ERR( dc1394_camera_enumerate(dcRef, &list), "Could not enumerate cameras" );

  if (i > list->num)
    throw CamException("Tried to get Guid of non-existant camera");

  uint64_t guid = list->ids[i].guid;

  dc1394_camera_free_list (list);

  return guid;
}

bool dc1394_cam::waitForData(int usec)
{
  return Cam::waitForData(usec);
}

bool
dc1394_cam::Cam::waitForData(int usec)
{
  struct timeval timeout;
  timeout.tv_sec = 0;     // If you want to sleep long than a second, you're probably doing something bad...
  timeout.tv_usec = usec;

  fd_set fds = camFds;

  int retval = select(FD_SETSIZE, &fds, NULL, NULL, &timeout);
  
  if (retval < 0)
  {
    fprintf(stderr, "Select() returned error: %s", strerror(errno));
  }

  if (retval <= 0)
    return false;

  return true;
}


dc1394_cam::Cam::Cam(uint64_t guid, 
                     dc1394speed_t speed, 
                     dc1394video_mode_t video,
                     dc1394framerate_t fps,
                     size_t bufferSize)        : started(false), dcCam(NULL)
{
  CHECK_READY();

  dcCam = dc1394_camera_new(dcRef, guid);

  if (!dcCam)
    throw CamException("Could not create camera");

  //  CHECK_ERR_CLEAN( dc1394_reset_bus(dcCam), "Could not reset bus" );

  CHECK_ERR_CLEAN( dc1394_video_set_iso_speed(dcCam, speed), "Could not set iso speed");
  
  CHECK_ERR_CLEAN( dc1394_video_set_mode(dcCam, video), "Could not set video mode");
  
  CHECK_ERR_CLEAN( dc1394_video_set_framerate(dcCam, fps), "Could not set framerate");

  CHECK_ERR_CLEAN( dc1394_capture_setup(dcCam, bufferSize, DC1394_CAPTURE_FLAGS_DEFAULT), "Could not setup camera.");

  FD_SET(dc1394_capture_get_fileno(dcCam), &camFds);
}



dc1394_cam::Cam::~Cam()
{
  if (dcCam != NULL)
    cleanup();
}



void
dc1394_cam::Cam::start()
{
  CHECK_READY();

  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_ON), "Could not start camera iso transmission");

  started = true;
}



void
dc1394_cam::Cam::stop()
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_video_set_transmission(dcCam, DC1394_OFF), "Could not stop camera iso transmission");

  started = false;
}


dc1394video_frame_t*
dc1394_cam::Cam::getFrame(dc1394capture_policy_t policy)
{
    CHECK_READY();

    if (!started)
      start();

    dc1394video_frame_t* f;
    CHECK_ERR_CLEAN( dc1394_capture_dequeue(dcCam, policy, &f), "Could not capture frame");
    return f;
}


void
dc1394_cam::Cam::releaseFrame(dc1394video_frame_t* f)
{
    CHECK_READY();
    CHECK_ERR_CLEAN( dc1394_capture_enqueue(dcCam, f), "Could not releaseframe");
}



void
dc1394_cam::Cam::setFeature(dc1394feature_t feature, uint32_t value)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present)
    CHECK_ERR_CLEAN( dc1394_feature_set_value(dcCam, feature, value), "Could not set feature");
}

void
dc1394_cam::Cam::setFeatureAbsolute(dc1394feature_t feature, float value)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present)
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON), "Could not enable absolute control.");
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_value(dcCam, feature, value), "Could not set feature");
  }
}

void
dc1394_cam::Cam::setFeatureMode(dc1394feature_t feature, dc1394feature_mode_t mode)
{
  CHECK_READY();
  dc1394bool_t present;
  CHECK_ERR_CLEAN( dc1394_feature_is_present(dcCam, feature, &present), "Could not check if feature was present");
  if (present)
  {
    CHECK_ERR_CLEAN( dc1394_feature_set_absolute_control(dcCam, feature,  DC1394_ON), "Could not disable absolute control.");
    CHECK_ERR_CLEAN( dc1394_feature_set_mode(dcCam, feature, mode), "Could not set feature");
  }
}


void
dc1394_cam::Cam::setControlRegister(uint64_t offset, uint32_t value)
{
  CHECK_READY();
  CHECK_ERR_CLEAN( dc1394_set_control_register(dcCam, offset, value), "Could not set control register");
}

uint32_t
dc1394_cam::Cam::getControlRegister(uint64_t offset)
{
  CHECK_READY();
  uint32_t value;
  CHECK_ERR_CLEAN( dc1394_get_control_register(dcCam, offset, &value), "Could not get control register");
  return value;
}


void
dc1394_cam::Cam::cleanup() {
  CHECK_READY();

  if (FD_ISSET(dc1394_capture_get_fileno(dcCam), &camFds))
    FD_CLR(dc1394_capture_get_fileno(dcCam), &camFds);

  dc1394_video_set_transmission(dcCam, DC1394_OFF);
  dc1394_capture_stop(dcCam);
  dc1394_camera_free(dcCam);
  
  dcCam = NULL;
}

