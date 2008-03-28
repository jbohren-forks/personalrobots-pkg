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
#include <iostream>
#include "axis_cam/axis_cam.h"
#include "string_utils/string_utils.h"

AxisCam::AxisCam(string ip) : ip(ip)
{
  jpeg_buf = NULL;
  jpeg_buf_size = 0;
  curl_global_init(0);

  ostringstream oss;
  oss << "http://" << ip << "/jpg/image.jpg";
  image_url = new char[oss.str().length()+1];
  strcpy(image_url, oss.str().c_str());

  oss.str(""); // clear it
  oss << "http://" << ip << "/axis-cgi/com/ptz.cgi";
  ptz_url = new char[oss.str().length()+1];
  strcpy(ptz_url, oss.str().c_str());

  jpeg_curl = curl_easy_init();
  curl_easy_setopt(jpeg_curl, CURLOPT_URL, image_url);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEFUNCTION, AxisCam::jpeg_write);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEDATA, this);

  getptz_curl = curl_easy_init();
  curl_easy_setopt(getptz_curl, CURLOPT_URL, ptz_url);
  curl_easy_setopt(getptz_curl, CURLOPT_WRITEFUNCTION, AxisCam::ptz_write);
  curl_easy_setopt(getptz_curl, CURLOPT_WRITEDATA, this);
  curl_easy_setopt(getptz_curl, CURLOPT_POSTFIELDS, "query=position");

  setptz_curl = curl_easy_init();
  curl_easy_setopt(setptz_curl, CURLOPT_URL, ptz_url);
  curl_easy_setopt(setptz_curl, CURLOPT_WRITEFUNCTION, AxisCam::ptz_write);
  curl_easy_setopt(setptz_curl, CURLOPT_WRITEDATA, this);

  printf("Getting images from [%s]\n", oss.str().c_str());
  if (!query_params())
    printf("sad! I couldn't query the camera parameters.\n");
  if (!query_params())
    printf("sad! I couldn't query the camera parameters.\n");
  if (!query_params())
    printf("sad! I couldn't query the camera parameters.\n");
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

size_t AxisCam::ptz_write(void *buf, size_t size, size_t nmemb, void *userp)
{
  if (size * nmemb == 0)
    return 0;
  AxisCam *a = (AxisCam *)userp;
  a->ptz_ss << string((char *)buf, size*nmemb);
  printf("%d bytes\n", size*nmemb);
  //cout << string((char *)buf, size*nmemb);
  return size*nmemb;
}

bool AxisCam::set_ptz(double pan, double tilt, double zoom, bool relative) 
{
  ostringstream oss;
  if (relative)
    oss << "rpan=" << pan 
        << "&rtilt=" << tilt
        << "&rzoom=" << zoom;
  else
    oss << "pan=" << clamp(pan, -175, 175)
        << "&tilt=" << clamp(tilt, -45, 90)
        << "&zoom=" << clamp(zoom, 0, 50000); // not sure of upper bound
  return send_params(oss.str());
}

int AxisCam::get_focus()
{
  if (last_autofocus_enabled)
  {
    set_focus(0, true); // manual focus but don't move it
    query_params();
    set_focus(0); // re-enable autofocus
    return last_focus;
  }
  query_params();
  return last_focus;
}
  
bool AxisCam::set_focus(int focus, bool relative)
{
  ostringstream oss;
  if (focus == 0 && !relative)
    oss << string("autofocus=on");
  else
  {
    last_autofocus_enabled = false;
    oss << string("autofocus=off&")
        << (relative ? "r" : "") << string("focus=") << focus;
  }
  return send_params(oss.str());
}

bool AxisCam::set_iris(int iris, bool relative)
{
  ostringstream oss;
  if (iris == 0)
    oss << "autoiris=on";
  else
    oss << string("autoiris=off&")
        << (relative ? "r" : "") << string("iris=") << iris;
  return send_params(oss.str());
}

bool AxisCam::send_params(string params)
{
  ptz_ss.str("");
  curl_easy_setopt(setptz_curl, CURLOPT_POSTFIELDS, params.c_str());
  CURLcode code;
  if (code = curl_easy_perform(setptz_curl))
  {
    printf("woah! curl error: [%s]\n", curl_easy_strerror(code));
    return false;
  }
  return true;
}

bool AxisCam::query_params()
{
  ptz_ss.str("");
  CURLcode code;
  if (code = curl_easy_perform(getptz_curl))
  {
    printf("woah! curl error: [%s]\n", curl_easy_strerror(code));
    return false;
  }
  printf("response:\n%s\n", ptz_ss.str().c_str());
  while (ptz_ss.good())
  {
    string line;
    getline(ptz_ss, line);
    vector<string> tokens;
    string_utils::split(line, tokens, "=");
    if (tokens.size() != 2)
      continue;
    if (tokens[0] == string("pan"))
      last_pan = atof(tokens[1].c_str());
    else if (tokens[0] == string("tilt"))
      last_tilt = atof(tokens[1].c_str());
    else if (tokens[0] == string("zoom"))
      last_zoom = atof(tokens[1].c_str());
    else if (tokens[0] == string("focus"))
      last_focus = atoi(tokens[1].c_str());
    else if (tokens[0] == string("iris"))
      last_iris = atoi(tokens[1].c_str());
    else if (tokens[0] == string("autofocus"))
      last_autofocus_enabled = (tokens[1] == string("on") ? true : false);
    else if (tokens[0] == string("autoiris"))
      last_autoiris_enabled = (tokens[1] == string("on") ? true : false);
/*
    printf("line has %d tokens:\n", tokens.size());
    for (int i = 0; i < tokens.size(); i++)
      printf(" [%s] ", tokens[i].c_str());
    printf("\n");
*/
  }
  return true;
}

