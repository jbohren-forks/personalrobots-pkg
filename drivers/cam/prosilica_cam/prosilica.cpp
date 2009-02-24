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
#include <PvRegIo.h>
#include <cassert>
#include <ctime>
#include <cstring>
#include <arpa/inet.h>

#define CHECK_ERR(fnc, amsg)                               \
do {                                                       \
  tPvErr err = fnc;                                        \
  if (err != ePvErrSuccess) {                              \
    char msg[256];                                         \
    snprintf(msg, 256, "%s: %s", amsg, errorStrings[err]); \
    throw ProsilicaException(msg);                         \
  }                                                        \
} while (false)

namespace prosilica {

static const unsigned int MAX_CAMERA_LIST = 10;
static const char* autoValues[] = {"Manual", "Auto", "AutoOnce"};
static const char* triggerModes[] = {"Freerun", "Software"};
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

  for (int tries = 0; tries < 5; ++tries)
  {
    cameraNum = PvCameraList(cameraList, MAX_CAMERA_LIST, NULL);
    if (cameraNum)
      return;
    usleep(1000000);
  }

  //throw ProsilicaException("Timed out looking for a camera");
  
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
  assert(i < MAX_CAMERA_LIST);
  if (i >= cameraNum)
    throw ProsilicaException("No camera at index i");
  return cameraList[i].UniqueId;
}

Camera::Camera(unsigned long guid, size_t bufferSize)
  : bufferSize_(bufferSize), mode_(None)
{
  CHECK_ERR( PvCameraOpen(guid, ePvAccessMaster, &handle_),
             "Unable to open requested camera" );
  setup();
}

Camera::Camera(const char* ip_address, size_t bufferSize)
  : bufferSize_(bufferSize), mode_(None)
{
  unsigned long addr = inet_addr(ip_address);
  CHECK_ERR( PvCameraOpenByAddr(addr, ePvAccessMaster, &handle_),
             "Unable to open requested camera" );
  setup();
}

void Camera::setup()
{
  // adjust packet size according to the current network capacity
  tPvUint32 maxPacketSize = 8228;
  PvAttrUint32Get(handle_, "PacketSize", &maxPacketSize);
  PvCaptureAdjustPacketSize(handle_, maxPacketSize);

  // query for attributes (TODO: more)
  CHECK_ERR( PvAttrUint32Get(handle_, "TotalBytesPerFrame", &frameSize_),
             "Unable to retrieve frame size" );
  
  // allocate frame buffers
  frames_ = new tPvFrame[bufferSize_];
  memset(frames_, 0, sizeof(tPvFrame) * bufferSize_);
  for (unsigned int i = 0; i < bufferSize_; ++i)
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

void Camera::setFrameCallback(boost::function<void (tPvFrame*)> callback)
{
  userCallback_ = callback;
}

void Camera::start(AcquisitionMode mode)
{
  assert( mode_ == None && mode != None );
  assert( mode == Triggered || !userCallback_.empty() );
  
  // set camera in acquisition mode
  CHECK_ERR( PvCaptureStart(handle_), "Could not start capture");

  // start capture after setting acquisition and trigger modes
  try {
    CHECK_ERR( PvAttrEnumSet(handle_, "AcquisitionMode", "Continuous"),
               "Could not set acquisition mode" );
    CHECK_ERR( PvAttrEnumSet(handle_, "FrameStartTriggerMode", triggerModes[mode]),
               "Could not set trigger mode" );
    CHECK_ERR( PvCommandRun(handle_, "AcquisitionStart"),
               "Could not start acquisition" );
  } catch (ProsilicaException& e) {
    PvCaptureEnd(handle_); // reset to non capture mode
    throw; // rethrow
  }

  if (mode != Triggered)
    for (unsigned int i = 0; i < bufferSize_; ++i)
      PvCaptureQueueFrame(handle_, frames_ + i, Camera::frameDone);

  mode_ = mode;
}

void Camera::stop()
{
  if (mode_ == None)
    return;
  
  PvCommandRun(handle_, "AcquisitionStop");
  PvCaptureEnd(handle_);
  PvCaptureQueueClear(handle_);
  mode_ = None;
}

tPvFrame* Camera::grab(unsigned long timeout_ms)
{
  assert( mode_ == Triggered );

  tPvFrame* frame = &frames_[0];
  CHECK_ERR( PvCaptureQueueFrame(handle_, frame, NULL), "Couldn't queue frame" );

  unsigned long time_so_far = 0;
  while (time_so_far < timeout_ms)
  {
    // trigger the camera
    CHECK_ERR( PvCommandRun(handle_, "FrameStartTriggerSoftware"),
               "Couldn't trigger capture" );

    // TODO: maybe can simplify this (avoid checking ePvErrTimeout?)
    // try to capture the frame
    tPvErr e;
    do
    {
      clock_t start_time = clock();
      e = PvCaptureWaitForFrameDone(handle_, frame, timeout_ms - time_so_far);
      if (timeout_ms != PVINFINITE)
        time_so_far += (clock() - start_time) / (CLOCKS_PER_SEC / 1000);
    } while (e == ePvErrTimeout && time_so_far < timeout_ms);

    if (e == ePvErrSuccess)
      return frame;

    // retry if data missing, probably no hope on other errors
    if (e != ePvErrDataMissing)
      return NULL;
  }
  
  return NULL;
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

unsigned long Camera::guid()
{
  unsigned long id;
  CHECK_ERR( PvAttrUint32Get(handle_, "UniqueId", &id),
             "Couldn't retrieve unique id" );
  return id;
}

static const unsigned long USER_ADDRESS = 0x17200;

void Camera::writeUserMemory(const char* data, size_t size)
{
  assert(size <= USER_MEMORY_SIZE);

  unsigned char buffer[USER_MEMORY_SIZE];
  unsigned long written;
  
  memset(buffer, 0, USER_MEMORY_SIZE);
  memcpy(buffer, data, size);
  
  CHECK_ERR( PvMemoryWrite(handle_, USER_ADDRESS, USER_MEMORY_SIZE, buffer, &written),
             "Couldn't write to user memory" );
}

void Camera::readUserMemory(char* data, size_t size)
{
  assert(size <= USER_MEMORY_SIZE);

  unsigned char buffer[USER_MEMORY_SIZE];
  
  CHECK_ERR( PvMemoryRead(handle_, USER_ADDRESS, USER_MEMORY_SIZE, buffer),
             "Couldn't read from user memory" );

  memcpy(data, buffer, size);
}

void Camera::frameDone(tPvFrame* frame)
{
  // don't requeue if capture has stopped
  if (frame->Status == ePvErrUnplugged || frame->Status == ePvErrCancelled)
    return;

  Camera* camPtr = (Camera*) frame->Context[0];
  if (frame->Status == ePvErrSuccess && camPtr && !camPtr->userCallback_.empty()) {
    // TODO: thread safety OK here?
    boost::lock_guard<boost::mutex> guard(camPtr->frameMutex_);
    camPtr->userCallback_(frame);
  }
  
  PvCaptureQueueFrame(camPtr->handle_, frame, Camera::frameDone);
}

} // namespace prosilica
