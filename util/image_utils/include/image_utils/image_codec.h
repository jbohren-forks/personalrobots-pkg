///////////////////////////////////////////////////////////////////////////////
// This template wraps MsgImage and friends to provide handy C++ functions to
// compress/decompress images.
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

#ifndef IMAGE_UTILS_IMAGE_CODEC_H
#define IMAGE_UTILS_IMAGE_CODEC_H

// the IJG JPEG library is not thread-safe, so we have to mutex calls to it
#include "image_utils/jpeg_wrapper.h"
#include <cstdio>
#include <string>
#include <assert.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
using std::string;

extern "C"
{
#include "jpeglib.h"
}

template<class T>
class ImageCodec
{
public:
  ImageCodec(T *_msg) : 
    msg(_msg), jpeg(NULL), raster(NULL), raster_alloc_size(0)
  {
  }

  virtual ~ImageCodec()
  {
    if (raster)
      delete[] raster;
    raster = NULL;
  }

  uint8_t *inflate(string colorspace = "rgb24")
  {
    msg->lock();
    // perform decompression and/or colorspace conversion if necessary
    if (msg->compression == string("jpeg"))
      decompress_jpeg();
    else if (msg->compression == string("raw"))
      decompress_raw(); // not really a decompression, but...
    else
    {
      printf("unimplemented image compression: [%s]\n", 
             msg->compression.c_str());
      msg->unlock();
      return NULL;
    }
    msg->unlock();
    return raster;
  }

  uint32_t get_raster_size()
  {
    int bpp = 0;
    if (msg->colorspace == "rgb24" || msg->colorspace == "bgr24")
      bpp = 3;
    return msg->width * msg->height * bpp;
  }

  bool deflate(int compression_quality = 90)
  {
    // TODO: perform decompression and/or colorspace conversion if necessary
    if (msg->compression == string("jpeg"))
      return compress_jpeg(compression_quality);
    else if (msg->compression == string("raw"))
      return compress_raw(); // not really compressing it, but...
    else
    {
      printf("unimplemented image compression: [%s]\n", 
             msg->compression.c_str());
      return false;
    }
    return true;
  }

protected:
  T *msg;
  JpegWrapper *jpeg;

  uint8_t *raster;
  uint32_t raster_alloc_size;

  bool compress_jpeg(int compression_quality)
  {
    if (msg->compression != string("jpeg"))
      return false;
    JpegWrapper::raster_type_t rt;
    if (msg->colorspace == "rgb24")
      rt = JpegWrapper::RT_RGB24;
    else
    {
      printf("image_codec::compress_jpeg() only handles colorspace 'rgb24'\n");
      return false;
    }
    if (!jpeg)
      jpeg = new JpegWrapper();
    uint32_t sz = jpeg->compress_to_jpeg(raster, 
                           msg->width, msg->height, rt, compression_quality);
    msg->set_data_size(sz);
    memcpy(msg->data, jpeg->get_compress_buf(), sz);
    return true;
  }

  bool compress_raw()
  {
    msg->set_data_size(get_raster_size());
    memcpy(msg->data, raster, get_raster_size());
    return true;
  }

  bool decompress_jpeg()
  {
    if (!jpeg)
      jpeg = new JpegWrapper();
    realloc_raster_if_needed();
    if (!jpeg->decompress_jpeg_buf((char *)msg->data, msg->get_data_size()))
      return false;
    memcpy(raster, jpeg->get_raster(), jpeg->raster_size());
    return true;
  }
  
  bool decompress_raw()
  {
    realloc_raster_if_needed();
    memcpy(raster, msg->data, msg->get_data_size());
    return true;
  }

  void realloc_raster_if_needed()
  {
    if (raster_alloc_size < get_raster_size())
    {
      if (raster)
        delete[] raster;
      raster_alloc_size = get_raster_size();
      raster = new uint8_t[raster_alloc_size];
    }
  }
};

#endif

