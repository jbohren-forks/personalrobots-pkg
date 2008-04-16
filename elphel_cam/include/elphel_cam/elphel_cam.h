///////////////////////////////////////////////////////////////////////////////
// The axis_cam package provides a library that talks to Axis IP-based cameras
// as well as ROS nodes which use these libraries
//
// Copyright (C) 2008, Morgan Quigley, Stanford Univerity
//                     Jeremy Leibs, Willow Garage
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice, 
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright 
//     notice, this list of conditions and the following disclaimer in the 
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University, Willow Garage, nor the names 
//     of its contributors may be used to endorse or promote products derived 
//     from this software without specific prior written permission.
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

#ifndef ELPHEL_CAM_ELPHEL_CAM_H
#define ELPHEL_CAM_ELPHEL_CAM_H

#include <curl/curl.h>
#include <string>
#include <sstream>
#include "thread_utils/mutex.h"

using namespace std;

class Elphel_Cam
{
public:
  Elphel_Cam(string ip);
  ~Elphel_Cam();

  bool next_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size);

  bool config_cmd(string url, string cmd);

  bool compressor_cmd(string cmd);
  bool ccam_cmd(string cmd);
  bool towp();

  bool start();
  bool stop();

  bool init(float = 10, int = 4, int = 4);

private:
  string ip;
  uint8_t *jpeg_buf, ;
  uint32_t jpeg_buf_size, jpeg_file_size;

  CURL *jpeg_curl, *config_curl;

  char *image_url, *towp_url, *ccam_url, *comp_url;

  stringstream config_ss;
  ThreadUtils::Mutex config_ss_mutex;

  static size_t jpeg_write(void *buf, size_t size, size_t nmemb, void *userp);
  static size_t config_write(void *buf, size_t size, size_t nmemb, void *userp);

};

#endif

