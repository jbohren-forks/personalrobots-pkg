///////////////////////////////////////////////////////////////////////////////
// The image_flows package provides image transport, codec wrapping, and 
// various handy image utilities.
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

#ifndef IMAGE_FLOWS_IMAGE_FLOW_CODEC_H
#define IMAGE_FLOWS_IMAGE_FLOW_CODEC_H

// the IJG JPEG library is not thread-safe, so we have to mutex calls to it
#include "ijg_libjpeg/ros_jpeg_mutex.h"
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
class ImageFlowCodec
{
public:
  ImageFlowCodec(T *flow) : 
    flow(flow), raster(NULL), raster_alloc_size(0)
  {
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    dinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&dinfo);
  }

  virtual ~ImageFlowCodec()
  {
    jpeg_destroy_compress(&cinfo);
    jpeg_destroy_decompress(&dinfo);
    if (raster)
      delete[] raster;
    raster = NULL;
  }

  uint8_t *get_raster(string colorspace = "rgb24")
  {
    if (flow->flow_dir == ROS_Flow::SOURCE)
    {
      realloc_raster_if_needed();
      return raster; // outbound is the easy case...
    }
    flow->lock_atom();
    // perform decompression and/or colorspace conversion if necessary
    if (flow->compression == string("jpeg"))
      decompress_jpeg();
    else
    {
      printf("unimplemented image compression: [%s]\n", flow->compression.c_str());
      flow->unlock_atom();
      return NULL;
    }
    flow->unlock_atom();
    return raster;
  }

  uint32_t get_raster_size()
  {
    int bpp = 0;
    if (flow->colorspace == "rgb24" || flow->colorspace == "bgr24")
      bpp = 3;
    return flow->width * flow->height * bpp;
  }

  bool write_file(string filename, int quality = 85, bool force_get_raster = true)
  {
    if (force_get_raster)
      get_raster(); // force a decompress to occur in case we haven't done it yet
    if (filename.substr(filename.size()-4) == string(".ppm"))
      return write_ppm_file(filename);
    else if (filename.substr(filename.size()-4) == string(".jpg"))
      return write_jpeg_file(filename, quality);
    printf("unknown file type\n");
    return false;
  }

  bool read_file(string filename)
  {
    if (filename.substr(filename.size()-4) == string(".jpg"))
      return read_jpeg_file(filename);
    printf("unknown file type\n");
    return false;
  }

  bool set_flow_data()
  {
    if (flow->flow_dir != ROS_Flow::SOURCE)
    {
      printf("hey! you can't send an image out a sink flow. UR DONE.\n");
      assert(0);
    }
    // perform decompression and/or colorspace conversion if necessary
    if (flow->compression == string("jpeg"))
      return compress_jpeg();
    else
    {
      printf("unimplemented image compression: [%s]\n", flow->compression.c_str());
      flow->unlock_atom();
      return false;
    }
    return true;
  }

private:
  T *flow;
  uint8_t *raster;
  uint32_t raster_alloc_size;
  jpeg_compress_struct cinfo;
  jpeg_decompress_struct dinfo;
  jpeg_error_mgr jerr;

  static void buffer_dest_init(j_compress_ptr cinfo) { }
  static void buffer_source_init(j_decompress_ptr dinfo) { }
  static boolean buffer_source_fill(j_decompress_ptr dinfo)
  {
    if (dinfo->src->bytes_in_buffer == 0)
    {
      printf("woah! the flow should have received the entire JPEG buffer...\n");
      return FALSE; // not sure what to do here...
    }
    return TRUE;
  }
  static void buffer_source_skip(j_decompress_ptr dinfo, long num_bytes)
  {
    dinfo->src->next_input_byte += (size_t)num_bytes;
    dinfo->src->bytes_in_buffer -= (size_t)num_bytes;
  }
  static void buffer_source_term(j_decompress_ptr dinfo) { }
  static void jpeg_buffer_src(j_decompress_ptr dinfo, char *buf, int size)
  {
    if (dinfo->src == NULL)
      dinfo->src = (struct jpeg_source_mgr *)(*dinfo->mem->alloc_small)
        ((j_common_ptr)dinfo, JPOOL_PERMANENT, sizeof(struct jpeg_source_mgr));
    dinfo->src->init_source = buffer_source_init;
    dinfo->src->fill_input_buffer = buffer_source_fill;
    dinfo->src->skip_input_data = buffer_source_skip;
    dinfo->src->resync_to_restart = jpeg_resync_to_restart;
    dinfo->src->next_input_byte = (JOCTET *)buf;
    dinfo->src->bytes_in_buffer = size;
    dinfo->src->term_source = buffer_source_term;
  }

  uint8_t *decompress_jpeg()
  {
    // sanity check
    if (flow->compression != string("jpeg"))
    {
      printf("woah! almost tried to use JPEG to uncompress a non-JPEG byte block\n");
      return NULL;
    }
    ros_jpeg_mutex_lock();
    jpeg_buffer_src(&dinfo, (char *)flow->data, flow->get_data_size());
    jpeg_read_header(&dinfo, TRUE);
    jpeg_start_decompress(&dinfo);
    if (dinfo.output_components != 3)
    {
      printf("woah! this jpeg byte block doesn't have 3 components.\n");
      printf("it says it has %d components. I don't know what to do...\n",
        dinfo.output_components);
      return NULL;
    }
    int row_stride = dinfo.output_width * dinfo.output_components;
    flow->width = dinfo.output_width; // override if necessary
    flow->height = dinfo.output_height;
    realloc_raster_if_needed();
    JSAMPROW row_pointer[1];
    while (dinfo.output_scanline < dinfo.output_height)
    {
      row_pointer[0] = (JSAMPROW)&raster[dinfo.output_scanline * row_stride];
      jpeg_read_scanlines(&dinfo, row_pointer, 1);
    }
    jpeg_finish_decompress(&dinfo);
    ros_jpeg_mutex_unlock();
    return raster;
  }

  bool compress_jpeg()
  {
    // TODO

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

  bool write_jpeg_file(string filename, int quality)
  {
    if (flow->colorspace != string("rgb24") && flow->colorspace != string("bgr24"))
    {
      printf("jpeg file writer can only handle RGB24 or BGR24 currently...\n");
      return false;
    }
    FILE *outfile;
    if ((outfile = fopen(filename.c_str(), "wb")) == NULL)
    {
      printf("can't open file for output: [%s]\n", filename.c_str());
      return false;
    }
    JSAMPLE *image_buffer = (JSAMPLE *)raster;
    struct jpeg_compress_struct cinfo; // declare a local one here...
    struct jpeg_error_mgr jerr;
    JSAMPROW row_pointer[1];
    ros_jpeg_mutex_lock();
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    jpeg_stdio_dest(&cinfo, outfile);
    cinfo.image_width = flow->width;
    cinfo.image_height = flow->height;
    cinfo.input_components = 3;
    cinfo.in_color_space = JCS_RGB;
    // TODO: flip byte ordering if we have a bgr raster
    jpeg_set_defaults(&cinfo);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = flow->width * 3;
    while (cinfo.next_scanline < cinfo.image_height)
    {
      row_pointer[0] = (JSAMPROW)&raster[cinfo.next_scanline * row_stride];
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);
    jpeg_destroy_compress(&cinfo);
    ros_jpeg_mutex_unlock();
    fclose(outfile);
    return true;
  }

  bool write_ppm_file(string filename)
  {
    if (flow->colorspace != string("rgb24") && flow->colorspace != string("bgr24"))
    {
      printf("ppm file writer can only handle RGB24 or BGR24 currently...\n");
      return false;
    }
    FILE *f = fopen(filename.c_str(), "wb");
    if (!f)
      return false;
    flow->lock_atom();
    fprintf(f, "P6%d %d\n255\n", flow->width, flow->height);
    if (flow->colorspace == string("rgb24"))
      fwrite(raster, 1, flow->width * flow->height * 3, f);
    else if (flow->colorspace == string("bgr24"))
    {
      uint8_t *bgr = new uint8_t[flow->width * flow->height * 3];
      for (int y = 0; y < flow->height; y++)
        for (int x = 0; x < flow->width; x++)
        {
          uint8_t *p = raster + y * flow->width * 3 + x * 3;
          uint8_t *q = bgr    + y * flow->width * 3 + x * 3;
          q[0] = p[2]; q[1] = p[1]; q[2] = p[0];
        }
      fwrite(bgr, 1, flow->width * flow->height * 3, f);
      delete[] bgr;
    }
    else
      assert(0);
    fclose(f);
    flow->unlock_atom();
  }

  bool read_jpeg_file(string filename)
  {
    // this function just reads in the jpeg block and puts it in the flow.
    // it just reads the jpeg header; it doesn't perform a full
    // decompression. that happens during get_raster.
    struct stat sbuf;
    if (stat(filename.c_str(), &sbuf))
    {
      printf("couldn't stat the file [%s]: (%d) %s\n", 
        filename.c_str(), errno, strerror(errno));
      return false;
    }
    flow->set_data_size(sbuf.st_size);
    FILE *f = fopen(filename.c_str(), "rb");
    if (!f)
    {
      printf("can't open %s for reading (%d): %s\n",
        filename.c_str(), errno, strerror(errno));
      return false;
    }
    int num_read = fread(flow->data, 1, sbuf.st_size, f);
    fclose(f);
    if (num_read != sbuf.st_size)
    {
      printf("woah! couldn't read the whole file.\n");
      return false;
    }
    flow->compression = "jpeg";
    flow->colorspace = "rgb24";
    // now, look again at the file header and see how big the image is
    struct jpeg_decompress_struct dinfo;
    f = fopen(filename.c_str(), "rb");
    ros_jpeg_mutex_lock();
    dinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&dinfo);
    jpeg_stdio_src(&dinfo, f);
    jpeg_read_header(&dinfo, TRUE);
    jpeg_start_decompress(&dinfo);
    flow->width = dinfo.output_width;
    flow->height = dinfo.output_height;
    jpeg_destroy_decompress(&dinfo);
    ros_jpeg_mutex_unlock();
    fclose(f);
    return true;
  }
};


#endif

