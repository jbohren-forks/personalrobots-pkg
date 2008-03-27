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

#include <sstream>
#include "axis_cam/axis_cam.h"

AxisCam::AxisCam(string ip) : ip(ip)
{
  jpeg_buf = NULL;
  jpeg_buf_size = 0;
  curl_global_init(0);

  ostringstream oss;
  oss << "http://" << ip << "/jpg/image.jpg";
  image_url = new char[oss.str().length()+1];
  strcpy(image_url, oss.str().c_str());

  jpeg_curl = curl_easy_init();
  curl_easy_setopt(jpeg_curl, CURLOPT_URL, image_url);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEFUNCTION, AxisCam::jpeg_write);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEDATA, this);

  printf("Getting images from [%s]", oss.str().c_str());
}

AxisCam::~AxisCam()
{
  delete[] image_url;
  if (jpeg_buf)
    delete[] jpeg_buf;
  jpeg_buf = NULL;
  curl_global_cleanup();
}

bool AxisCam::get_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size)
{
  if (fetch_jpeg_buf && fetch_buf_size)
  {
    *fetch_jpeg_buf = NULL;
    *fetch_buf_size = 0;
  }
  else
  {
    printf("woah! bad input parameters\n");
    return false; // don't make me crash
  }
  jpeg_file_size = 0;
  CURLcode code;
  if (code = curl_easy_perform(jpeg_curl))
  {
    printf("woah! curl error: [%s]\n", curl_easy_strerror(code));
    return false;
  }
  *fetch_jpeg_buf = jpeg_buf;
  *fetch_buf_size = jpeg_file_size;
  return true;
}

size_t AxisCam::jpeg_write(void *buf, size_t size, size_t nmemb, void *userp)
{
  if (size * nmemb == 0)
    return 0;
  AxisCam *a = (AxisCam *)userp;
  if (a->jpeg_file_size + size*nmemb >= a->jpeg_buf_size)
  {
    // overalloc
    a->jpeg_buf_size = 2 * (a->jpeg_file_size + (size*nmemb));
    printf("jpeg_buf_size is now %d\n", a->jpeg_buf_size);
    if (a->jpeg_buf)
      delete[] a->jpeg_buf;
    a->jpeg_buf = new uint8_t[a->jpeg_buf_size];
  }
  memcpy(a->jpeg_buf + a->jpeg_file_size, buf, size*nmemb);
  a->jpeg_file_size += size*nmemb;
  return size*nmemb;
}
  
  /*
  if (jpeg_file_size > jpeg_buf_size)
  {
    if (jpeg_buf)
      delete[] jpeg_buf;
    jpeg_buf_size = (uint32_t)(1.5*jpeg_file_size);
    jpeg_buf = new uint8_t[jpeg_buf_size]; // go big to save reallocs
  }
  size_t bytes_read = fread(jpeg_buf, 1, jpeg_file_size, jpeg_file);
  if (bytes_read != jpeg_file_size)
  {
    printf("couldn't read entire jpeg file\n");
    return false;
  }
  fclose(jpeg_file);
  *fetch_jpeg_buf = jpeg_buf;
  *fetch_buf_size = jpeg_file_size;
  */

bool AxisCam::ptz(double pan, double tilt, double zoom)
{
  pan = clamp(pan, -175, 175);
  tilt = clamp(tilt, -45, 90);
  zoom = clamp(zoom, 0, 50000); // not sure of the real upper bound. units are rather magical.
  ostringstream oss;
  oss << string("wget -q -O/dev/null \"") << ip << string("/axis-cgi/com/ptz.cgi?camera=1&pan=") << pan 
      << string("&tilt=") << tilt << string("&zoom=") << zoom << string("\"");
  //printf("about to execute: [%s]\n", oss.str().c_str());
  int retval = system(oss.str().c_str());
  if (retval > 0)
  {
    printf("ahhh nonzero return value from wget during ptz: %d\n", retval);
    return false;
  }
  else if (retval < 0)
  {
    // not sure what's happening here, but it appears to be benign.
    //printf("odd wget system retval = %d\n", retval);
  }
  return true;
}

