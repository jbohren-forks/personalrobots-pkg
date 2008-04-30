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
#include "elphel_cam/elphel_cam.h"

Elphel_Cam::Elphel_Cam(string ip) : ip(ip)
{
  jpeg_buf = NULL;
  jpeg_buf_size = 0;
  curl_global_init(0);

  //URL to get next image, and increment pointer
  ostringstream oss; 
  oss << "http://" << ip << ":8081/torp/wait/img/next/save";
  image_url = new char[oss.str().length()+1];
  strcpy(image_url, oss.str().c_str());

  //URL to initialize pointer
  oss.str("");
  oss << "http://" << ip << ":8081/towp/wait/next/save";
  towp_url = new char[oss.str().length()+1];
  strcpy(towp_url, oss.str().c_str());

  //URL to control camera adjustments
  oss.str("");
  oss << "http://" << ip << "/admin-bin/ccam.cgi?";
  ccam_url = new char[oss.str().length()+1];
  strcpy(ccam_url, oss.str().c_str());

  //URL to turn on/off the hardware compressor
  oss.str("");
  oss << "http://" << ip << ":81/compressor.php?";
  comp_url = new char[oss.str().length()+1];
  strcpy(comp_url, oss.str().c_str());

  jpeg_curl = curl_easy_init();
  curl_easy_setopt(jpeg_curl, CURLOPT_URL, image_url);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEFUNCTION, Elphel_Cam::jpeg_write);
  curl_easy_setopt(jpeg_curl, CURLOPT_WRITEDATA, this);

  config_curl = curl_easy_init();
  curl_easy_setopt(config_curl, CURLOPT_WRITEFUNCTION, Elphel_Cam::config_write);
  curl_easy_setopt(config_curl, CURLOPT_WRITEDATA, this);
  
}

Elphel_Cam::~Elphel_Cam()
{
  delete[] image_url;
  delete[] towp_url;
  delete[] ccam_url;
  delete[] comp_url;
  if (jpeg_buf)
    delete[] jpeg_buf;
  jpeg_buf = NULL;
  curl_global_cleanup();
}

bool Elphel_Cam::next_jpeg(uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size)
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
  CURLcode code;
  do
  {
    jpeg_file_size = 0;
    if (code = curl_easy_perform(jpeg_curl))
    {
      printf("woah! curl error: [%s]\n", curl_easy_strerror(code));
      return false;
    }
    if (jpeg_buf[0] == 0 && jpeg_buf[1] == 0)
      printf("[Elphel_Cam] ODD...first two bytes are zero...\n");
  } while (jpeg_buf[0] == 0 && jpeg_buf[1] == 0);
  *fetch_jpeg_buf = jpeg_buf;
  *fetch_buf_size = jpeg_file_size;
  return true;
}


bool Elphel_Cam::config_cmd(string url, string cmd)
{
  config_ss_mutex.lock();
  config_ss.clear(); // reset stringstream state so we can insert into it again
  config_ss.str("");
  config_ss_mutex.unlock();

  ostringstream oss;
  oss << url << cmd;

  char urlbuf[512];
  strcpy(urlbuf, oss.str().c_str());

  curl_easy_setopt(config_curl, CURLOPT_URL, urlbuf);

  CURLcode code;
  if (code = curl_easy_perform(config_curl))
  {
    printf("woah! curl error: [%s]\n", curl_easy_strerror(code));
    return false;
  }
  return true;
}


bool Elphel_Cam::compressor_cmd(string cmd) {
  return config_cmd(comp_url, cmd);
}

bool Elphel_Cam::ccam_cmd(string cmd) {
  return config_cmd(ccam_url, cmd);
}

bool Elphel_Cam::towp() {
  return config_cmd(towp_url, string(""));
}

bool Elphel_Cam::init(float fps, int im_dec, int im_bin) {

  uint8_t *jpeg;
  uint32_t jpeg_size;

  // We loop twice here and grab 10 images per loop This is fairly
  // heuristic and involves the fact that the autogain stuff doesn't
  // work right until the camera is actually capturing images.  We
  // want this to work if the camera has just powered on without being
  // manually configured.
  for (int i = 0; i < 2; i++) {

    if (!compressor_cmd("cmd=reset"))
      return false;

    //These are the dark fairly undocumented incantations that
    //initialize the camera well:
    //"Documentation" can be found at: http://wiki.elphel.com/index.php?title=Ccam.cgi
    ostringstream oss;
    oss << "opt=vhcxyu+!-"
	<< "&dh=" << im_dec << "&dv=" << im_dec
	<< "&bh=" << im_bin << "&bh=" << im_bin
	<< "&iq=90"
	<< "&fps=" << fps << ".0&fpslm=3"
	<< "&sens=4&bit=8&gam=50&pxl=10&csb=200&csr=200&rscale=auto&bscale=auto";
    
    if (!ccam_cmd(oss.str()))
      return false;

    if (!compressor_cmd("cmd=run"))
      return false;

    if (!towp())
      return false;

    for (int j = 0; j < 10;j++) {
      if (!next_jpeg(&jpeg, &jpeg_size))
	return false;
    }

    if (!compressor_cmd("cmd=stop"))
      return false;
  }

  return true;

}


bool Elphel_Cam::start() {
  
  //We put in sleeps here (*cringe*) because otherwise if we start reading too fast things break.

  usleep(100000);

  if (!compressor_cmd("cmd=run"))
    return false;
  
  usleep(100000);

  if (!towp())
    return false;

  usleep(100000);

  return true;

}

bool Elphel_Cam::stop() {
  return compressor_cmd("cmd=stop");
}


size_t Elphel_Cam::jpeg_write(void *buf, size_t size, size_t nmemb, void *userp)
{
  if (size * nmemb == 0)
    return 0;
  Elphel_Cam *a = (Elphel_Cam *)userp;
  if (a->jpeg_file_size + size*nmemb >= a->jpeg_buf_size)
  {
    // overalloc
    a->jpeg_buf_size = 2 * (a->jpeg_file_size + (size*nmemb));
    if (a->jpeg_buf)
      delete[] a->jpeg_buf;
    a->jpeg_buf = new uint8_t[a->jpeg_buf_size];
  }
  memcpy(a->jpeg_buf + a->jpeg_file_size, buf, size*nmemb);
  a->jpeg_file_size += size*nmemb;
  return size*nmemb;
}


size_t Elphel_Cam::config_write(void *buf, size_t size, size_t nmemb, void *userp)
{
  if (size * nmemb == 0)
    return 0;
  Elphel_Cam *a = (Elphel_Cam *)userp;
  a->config_ss_mutex.lock();
  a->config_ss << string((char *)buf, size*nmemb);
  a->config_ss_mutex.unlock();
  return size*nmemb;
}
