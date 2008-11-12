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

#include <cstring>
#include "curltest.h"

#define RETURN_CURL_ERR(curl, code) \
      fprintf(stderr, "curl error: [%s]\n", curl_easy_strerror(code)); \
      if (code == 22) \
      { \
        int http_code; \
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code); \
        fprintf(stderr, "HTTP response: %d\n", http_code); \
        return http_code; \
      } \
      return code; \

CurlTest::CurlTest(string ip) : ip(ip)
{
  buf = NULL;
  buf_size = 0;

  ostringstream oss;

  oss << "http://" << ip << "/axis-cgi/com/ptz.cgi";
  url = new char[oss.str().length()+1];
  strcpy(url, oss.str().c_str());

  curl_global_init(0);

  get_curl = curl_easy_init();

  curl_easy_setopt(get_curl, CURLOPT_URL, url);
  curl_easy_setopt(get_curl, CURLOPT_WRITEFUNCTION, CurlTest::buf_write);
  curl_easy_setopt(get_curl, CURLOPT_WRITEDATA, this);
  curl_easy_setopt(get_curl, CURLOPT_POSTFIELDS, "query=position");
  curl_easy_setopt(get_curl, CURLOPT_TIMEOUT, 1);
  curl_easy_setopt(get_curl, CURLOPT_FAILONERROR, 1);

  buf_size = 100;

  buf = new uint8_t[buf_size];
  memset(buf, 0, buf_size);
}

CurlTest::~CurlTest()
{
  if (buf)
    delete[] buf;

  buf = NULL;
  curl_global_cleanup();
}

size_t CurlTest::buf_write(void *buf, size_t size, size_t nmemb, void *userp)
{
  if (size * nmemb == 0)
    return 0;

  CurlTest *a = (CurlTest *)userp;

  if (a->buf_file_size + size*nmemb >= a->buf_size)
  {
    a->buf_size = 2 * (a->buf_file_size + (size*nmemb));

    uint8_t* tmp = new uint8_t[a->buf_size];

    if (a->buf)
    {
      memcpy(tmp, a->buf, a->buf_file_size);
      delete[] a->buf;
    } else {
      memset(tmp, 0, a->buf_size);
    }
    a->buf = tmp;
  }
  memcpy(a->buf + a->buf_file_size, buf, size*nmemb);
  a->buf_file_size += size*nmemb;
  return size*nmemb;
}

int CurlTest::get()
{

  CURLcode code;

  buf_file_size = 0;

  if ((code = curl_easy_perform(get_curl)))
  {
    RETURN_CURL_ERR(get_curl, code);
  }

  printf("Read: %s\n", buf);

  return 0;
}
