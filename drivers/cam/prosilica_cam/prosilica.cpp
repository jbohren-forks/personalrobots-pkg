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

#include "prosilica.h"

#define CHECK_ERR(fnc, amsg)                               \
do {                                                       \
  tPvErr err = fnc;                                        \
  if (err != ePvErrSuccess) {                              \
    char msg[256];                                         \
    snprintf(msg, 256, "%s: %s", errorStrings[err], amsg); \
    throw ProsilicaException(msg);                         \
  }                                                        \
} while (false)

namespace prosilica {

static const int MAX_CAMERA_LIST = 10;
static const char* autoValues[] = {"Manual", "Auto", "AutoOnce"};
static const char* errorStrings[] = {"No error",
                                     "Unexpected camera fault",
                                     "Unexpected fault in PvApi or driver",
                                     "Camera handle is invalid",
                                     "Bad parameter to API call",
                                     "Sequence of API calls is incorrect",
                                     "Camera or attribute not found",
                                     "Camera cannot be opened in the specified mode",
                                     "Camera was unplugged",
                                     "Setup is invalid (an attribute is invalid)",
                                     "System/network resources or memory not available",
                                     "1394 bandwidth not available",
                                     "Too many frames on queue",
                                     "Frame buffer is too small",
                                     "Frame cancelled by user",
                                     "The data for the frame was lost",
                                     "Some data in the frame is missing",
                                     "Timeout during wait",
                                     "Attribute value is out of the expected range",
                                     "Attribute is not this type (wrong access function)",
                                     "Attribute write forbidden at this time",
                                     "Attribute is not available at this time",
                                     "A firewall is blocking the traffic"};

static tPvCameraInfo cameraList[MAX_CAMERA_LIST];
static unsigned long cameraNum = 0;

void init()
{
  CHECK_ERR( PvInitialize(), "Failed to initialize Prosilica API" );

  // TODO: should timeout after a while
  while (true)
  {
    cameraNum = PvCameraList(cameraList, MAX_CAMERA_LIST, NULL);
    if (cameraNum)
      break;
    usleep(1000000);
  }

  // TODO: Callbacks for add/remove camera?
}

void fini()
{
  PvUnInitialize();
}

size_t numCameras()
{
  return cameraNum;
}

uint64_t getGuid(size_t i)
{
  // TODO: error checking
  return cameraList[i].UniqueId;
}

Camera::Camera(uint64_t guid, size_t bufferSize)
  : bufferSize_(bufferSize)
{
  CHECK_ERR( PvCameraOpen(guid, ePvAccessMaster, &handle_), "Unable to open requested camera" );

  // set pixel format
  CHECK_ERR( PvAttrEnumSet(handle_, "PixelFormat", "Rgb24"), "Unable to set pixel format" );
  
  // query for attributes (TODO: more)
  CHECK_ERR( PvAttrUint32Get(handle_, "TotalBytesPerFrame", &frameSize_),
             "Unable to retrieve frame size" );
  
  // allocate frame buffers
  frames_ = new tPvFrame[bufferSize];
  memset(frames_, 0, sizeof(tPvFrame) * bufferSize);
  for (unsigned int i = 0; i < bufferSize; ++i)
  {
    frames_[i].ImageBuffer = new char[frameSize_];
    frames_[i].ImageBufferSize = frameSize_;
    frames_[i].Context[0] = (void*)this; // for frameDone callback
  }
}

Camera::~Camera()
{
  stop();
  
  PvCameraClose(handle_);

  if (frames_)
  {
    for (unsigned int i = 0; i < bufferSize_; ++i)
      delete[] (char*)frames_[i].ImageBuffer;
    delete[] frames_;
  }
}

// TODO: this is kind of wacky... using extra layer of indirection for callbacks
//       seems necessary though due to Prosilica's slightly weird callback interface
void Camera::setFrameCallback(boost::function<void (tPvFrame*)> callback)
{
  userCallback_ = callback;
}

void Camera::start()
{
  if (userCallback_.empty())
    throw ProsilicaException("Must set frame callback before calling start()");
  
  // set camera in acquisition mode
  CHECK_ERR( PvCaptureStart(handle_), "Could not start camera");

  // set the acquisition mode to continuous
  if ( PvAttrEnumSet(handle_, "AcquisitionMode", "Continuous") ||
       PvAttrEnumSet(handle_, "FrameStartTriggerMode", "Freerun") ||
       PvCommandRun(handle_, "AcquisitionStart") )
  {
    PvCaptureEnd(handle_); // reset to non capture mode
    throw ProsilicaException("Could not set acquisition mode\n");
  }

  for (unsigned int i = 0; i < bufferSize_; ++i)
    PvCaptureQueueFrame(handle_, frames_ + i, Camera::frameDone);
}

void Camera::stop()
{
  PvCommandRun(handle_, "AcquisitionStop");
  PvCaptureEnd(handle_);
  PvCaptureQueueClear(handle_);
}

void Camera::setExposure(unsigned int val, AutoSetting isauto)
{
  CHECK_ERR( PvAttrEnumSet(handle_, "ExposureMode", autoValues[isauto]),
             "Couldn't set exposure mode" );

  if (isauto == Manual)
    CHECK_ERR( PvAttrUint32Set(handle_, "ExposureValue", val),
               "Couldn't set exposure value" );
}

void Camera::setGain(unsigned int val, AutoSetting isauto)
{
  CHECK_ERR( PvAttrEnumSet(handle_, "GainMode", autoValues[isauto]),
             "Couldn't set gain mode" );

  if (isauto == Manual)
    CHECK_ERR( PvAttrUint32Set(handle_, "GainValue", val),
               "Couldn't set gain value" );
}

void Camera::setWhiteBalance(unsigned int blue, unsigned int red, AutoSetting isauto)
{
  CHECK_ERR( PvAttrEnumSet(handle_, "WhitebalMode", autoValues[isauto]),
             "Couldn't set white balance mode" );

  if (isauto == Manual)
  {
    CHECK_ERR( PvAttrUint32Set(handle_, "WhitebalValueBlue", blue),
               "Couldn't set white balance blue value" );
    CHECK_ERR( PvAttrUint32Set(handle_, "WhitebalValueRed", red),
               "Couldn't set white balance red value" );
  }
}

void Camera::frameDone(tPvFrame* frame)
{
  // don't requeue if capture has stopped
  if (frame->Status == ePvErrUnplugged || frame->Status == ePvErrCancelled)
    return;

  Camera* camPtr = (Camera*) frame->Context[0];
  if (frame->Status == ePvErrSuccess && camPtr) {
    // TODO: thread safety OK here?
    boost::lock_guard<boost::mutex> guard(camPtr->frameMutex_);
    camPtr->userCallback_(frame);
  }
  
  PvCaptureQueueFrame(camPtr->handle_, frame, Camera::frameDone);
}

} // namespace prosilica
