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

#ifndef PROSILICA_H
#define PROSILICA_H

#include <stdexcept>
#include <boost/function.hpp>

// PvApi.h is blissfully unaware of the usual detection macros
// TODO: do this properly
//       beware: Prosilica's sized types seem screwed up
#define _LINUX
#define _x86
#include <PvApi.h>

namespace prosilica {

struct ProsilicaException : public std::runtime_error
{
  ProsilicaException(const char* msg) : std::runtime_error(msg) {}
};

void init();                // initializes API
void fini();                // releases internal resources
size_t numCameras();        // number of cameras found
uint64_t getGuid(size_t i); // camera ids

class Camera
{
public:
  Camera(uint64_t guid, size_t bufferSize = 8);

  ~Camera();

  void setFrameCallback(boost::function<void (tPvFrame*)> callback);
  void start();
  void stop();

private:
  tPvHandle handle_; // handle to open camera
  tPvFrame* frames_; // array of frame buffers
  tPvUint32 frameSize_; // bytes per frame
  size_t bufferSize_; // number of frame buffers
  boost::function<void (tPvFrame*)> userCallback_;

  static void frameDone(tPvFrame* frame);
};

} // namespace prosilica

#endif
