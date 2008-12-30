///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its 
//     contributors may be used to endorse or promote products derived from 
//     this software without specific prior written permission.
//   
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.

#ifndef AXIS_CAM_AXIS_CAM_H
#define AXIS_CAM_AXIS_CAM_H

#include <curl/curl.h>
#include <string>
#include <sstream>
#include "boost/thread/mutex.hpp"
using namespace std;

class AxisCam
{
public:
  AxisCam(string ip);
  ~AxisCam();
  
  void set_host(string ip);

  int get_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size);
  int set_ptz(double pan, double tilt, double zoom, bool relative = false);
  int set_focus(int focus = 0, bool relative = false); // zero for autofocus
  int get_focus(int*);
  int set_iris(int iris = 0, bool relative = false, bool blocking = true);
  int get_iris(int*);

  int send_params(string params);
  int query_params();

  void print_params();

  int last_iris, last_focus;
  double last_pan, last_tilt, last_zoom;
  bool last_autofocus_enabled, last_autoiris_enabled;

private:
  string ip;
  uint8_t *jpeg_buf;
  uint32_t jpeg_buf_size, jpeg_file_size;
  CURL *jpeg_curl, *getptz_curl, *setptz_curl;
  char *image_url, *ptz_url;
  stringstream ptz_ss; // need to mutex this someday...
  boost::mutex ptz_ss_mutex;
  inline double clamp(double d, double low, double high)
  { 
    return (d < low ? low : (d > high ? high : d)); 
  }
  static size_t jpeg_write(void *buf, size_t size, size_t nmemb, void *userp);
  static size_t ptz_write(void *buf, size_t size, size_t nmemb, void *userp);

};

#endif

