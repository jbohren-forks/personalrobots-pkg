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

#include <iostream>
#include <sstream>
#include <sys/stat.h>
#include <curl/curl.h>

#include "axis213/axis213.h"


Axis213::Axis213(string ip) : ip(ip)
{
  jpeg_buf_size = 5000000;
  ptz_buf_size = 1000;

  jpeg_buf = new uint8_t[jpeg_buf_size]; // go big to save reallocs
  ptz_buf = new char[ptz_buf_size];

  jpeg_file_size = 0;
  ptz_file_size = 0;

  curl_global_init(0);

  ostringstream oss;
  oss << "http://" << ip << "/axis-cgi/com/ptz.cgi";
  strcpy(ptz_control_url, oss.str().c_str());

  req_count = 0;
  ptz_handle_control = curl_easy_init();
  curl_easy_setopt(ptz_handle_control, CURLOPT_URL, ptz_control_url);
  curl_easy_setopt(ptz_handle_control, CURLOPT_WRITEFUNCTION, Axis213::write_ptz);
  curl_easy_setopt(ptz_handle_control, CURLOPT_WRITEDATA, this);

  oss.str("");
  oss << "http://" << ip << "/axis-cgi/com/ptz.cgi";
  strcpy(ptz_observe_url, oss.str().c_str());

  ptz_handle_observe = curl_easy_init();
  curl_easy_setopt(ptz_handle_observe, CURLOPT_URL, ptz_observe_url);
  curl_easy_setopt(ptz_handle_observe, CURLOPT_WRITEFUNCTION, Axis213::write_ptz);
  curl_easy_setopt(ptz_handle_observe, CURLOPT_WRITEDATA, this);
  curl_easy_setopt(ptz_handle_observe, CURLOPT_POSTFIELDS, "query=position");

  oss.str("");
  oss << "http://" << ip << "/jpg/image.jpg";
  strcpy(image_url, oss.str().c_str());

  image_handle = curl_easy_init();
  curl_easy_setopt(image_handle, CURLOPT_URL, image_url);
  curl_easy_setopt(image_handle, CURLOPT_WRITEFUNCTION, Axis213::write_jpeg);
  curl_easy_setopt(image_handle, CURLOPT_WRITEDATA, this);

  std::cout << "Getting images from: " << image_url << std::endl;

}

Axis213::~Axis213()
{
  if (jpeg_buf)
    delete[] jpeg_buf;
  if (ptz_buf)
    delete[] ptz_buf;
  jpeg_buf = NULL;
  ptz_buf = NULL;
  curl_global_cleanup();
}

bool Axis213::get_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size)
{
  std::cout << "Fetching jpeg." << std::endl;
  if (fetch_jpeg_buf && fetch_buf_size)
  {
    *fetch_jpeg_buf = NULL;
    *fetch_buf_size = 0;
  }
  
  jpeg_file_size = 0;

  if (curl_easy_perform(image_handle))
  {
    return false;
  } else {
    *fetch_jpeg_buf = jpeg_buf;
    *fetch_buf_size = jpeg_file_size;
    return true;
  }  
}

bool Axis213::get_ptz(float* pan, float* tilt, float* zoom, float* focus)
{
  ptz_file_size = 0;

  if (curl_easy_perform(ptz_handle_observe))
  {
    return false;
  } else {
    ptz_buf[ptz_file_size] = 0;

    istringstream response(ptz_buf);
    string line;
    int ind;
    while (getline(response, line)) {
      ind = line.find_first_of('=');
      istringstream iss(line.substr(ind+1,line.length()));
      if (line.compare(0,ind,"pan") == 0) {
	iss >> *pan;
      }
      if (line.compare(0,ind,"tilt") == 0) {
	iss >> *tilt;
      }
      if (line.compare(0,ind,"zoom") == 0) {
	iss >> *zoom;
      }
      if (line.compare(0,ind,"focus") == 0) {
	iss >> *focus;
      }
      if (line.compare(0,ind,"autofocus") == 0) {
	if (iss.str().compare(0,2,"on") == 0) {	
	  *focus = -1.0;
	}
      }
    }
    return true;
  }
  
}

bool Axis213::ptz(float pan, float tilt, float zoom, float focus, bool relative)
{


  ostringstream oss;

  if (relative) {
    oss << string("rpan=") << pan
	<< string("&rtilt=") << tilt 
	<< string("&rzoom=") << zoom
	<< string("&rfocus=") << focus;
  } else {
    pan = clamp(pan, -169, 169);
    tilt = clamp(tilt, -10, 90);
    zoom = clamp(zoom, 0, 50000); 
    oss << string("pan=") << pan
	<< string("&tilt=") << tilt 
	<< string("&zoom=") << zoom;
    if (focus >= 1) {
      oss << string("&autofocus=off")
	  << string("&focus=") << focus;
    } else if (focus < 0) {
      oss << string("&autofocus=on");
    } else {
      oss << string("&autofocus=off");
    }
  }
 
  std::cout << req_count++ << " Sending command: " << oss.str() << std::endl;


  char postfield[256];

  strcpy(postfield, oss.str().c_str());

  ptz_file_size = 0;

  curl_easy_setopt(ptz_handle_control, CURLOPT_POSTFIELDS, postfield);
  
  if (curl_easy_perform(ptz_handle_control))
  {
    return false;
  } else {
    return true;
  }
}


size_t Axis213::write_ptz(void* buffer, size_t size, size_t nmemb, void* userp) {
  if (size*nmemb == 0) {
    return 0;
  }

  Axis213* a = (Axis213*)userp;

  if (a->ptz_file_size + size*nmemb + 1 < a->ptz_buf_size) {

    memcpy(a->ptz_buf + a->ptz_file_size, buffer, size*nmemb);

    a->ptz_file_size += size*nmemb;

    return size*nmemb;

  } else {

    return 0;

  }
}

size_t Axis213::write_jpeg(void* buffer, size_t size, size_t nmemb, void* userp) {
  if (size*nmemb == 0) {
    return 0;
  }

  Axis213* a = (Axis213*)userp;

  if (a->jpeg_file_size + size*nmemb < a->jpeg_buf_size) {

    memcpy(a->jpeg_buf + a->jpeg_file_size, buffer, size*nmemb);

    a->jpeg_file_size += size*nmemb;

    return size*nmemb;
  
  } else {

    return 0;

  }
}
