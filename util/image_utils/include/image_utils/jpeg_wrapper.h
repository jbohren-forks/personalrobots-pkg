#ifndef IMAGE_UTILS_JPEG_WRAPPER_H
#define IMAGE_UTILS_JPEG_WRAPPER_H

#include <inttypes.h>
#include "ijg_libjpeg/ros_jpeg_mutex.h"
extern "C"
{
#include "ijg_libjpeg/jpeglib.h"
}
#include <cstdlib>
#include <cstring>

class JpegWrapper
{
public:
  static JpegWrapper *g_wrapper;
  enum raster_type_t
  {
    RT_RGB24,
    RT_MONO8
  };

  JpegWrapper() 
  : raster(NULL), compress_buf(NULL),
    raster_width(0), raster_height(0),
    raster_alloc_size(0), compress_buf_alloc_size(0)
  {
    g_wrapper = this;
    cinfo.err = jpeg_std_error(&jerr);
    jpeg_create_compress(&cinfo);
    dinfo.err = jpeg_std_error(&jerr);
    jpeg_create_decompress(&dinfo);
  }

  ~JpegWrapper()
  {
    jpeg_destroy_compress(&cinfo);
    jpeg_destroy_decompress(&dinfo);
    if (raster)
      delete[] raster;
    if (compress_buf)
      delete[] compress_buf;
  }

  inline int width()  { return raster_width; }
  inline int height() { return raster_height; }
  inline int raster_size() { return 3 * raster_width * raster_height; }
  inline uint8_t *get_raster() { return raster; }

  bool decompress_jpeg_buf(char *data, uint32_t data_len)
  {
    ros_jpeg_mutex_lock();
    jpeg_buffer_src(&dinfo, data, data_len);
    jpeg_read_header(&dinfo, TRUE);
    jpeg_start_decompress(&dinfo);
    if (dinfo.output_components != 3)
    {
      printf("woah! this jpeg buffer doesn't have three components.\n");
      return false;
    }
    raster_width = dinfo.output_width;
    raster_height = dinfo.output_height;
    realloc_raster_if_needed();
    int row_stride = raster_width * 3;
    JSAMPROW row_pointer[1];
    while (dinfo.output_scanline < dinfo.output_height)
    {
      row_pointer[0] = (JSAMPROW)&raster[dinfo.output_scanline * row_stride];
      jpeg_read_scanlines(&dinfo, row_pointer, 1);
    }
    jpeg_finish_decompress(&dinfo);
    ros_jpeg_mutex_unlock();
    return true;
  }
  uint32_t compress_to_jpeg(uint8_t *raster, uint32_t w, uint32_t h, 
                            raster_type_t raster_type = RT_RGB24, 
                            int32_t quality = 95)
  {
    cinfo.image_width = w;
    cinfo.image_height = h;
    if (raster_type == RT_RGB24)
    {
      cinfo.input_components = 3;
      cinfo.in_color_space = JCS_RGB;
    }
    else if (raster_type == RT_MONO8)
    {
      cinfo.input_components = 1;
      cinfo.in_color_space = JCS_GRAYSCALE;
    }
    const int compress_start_size = 1024;
    if (!compress_buf)
    {
      compress_buf = new uint8_t[compress_start_size];
      compress_buf_alloc_size = compress_start_size;
    }
    ros_jpeg_mutex_lock();
    jpeg_set_defaults(&cinfo);
    jpeg_buffer_dest(&cinfo, (char *)compress_buf, compress_buf_alloc_size);
    jpeg_set_quality(&cinfo, quality, TRUE);
    jpeg_start_compress(&cinfo, TRUE);
    int row_stride = w * cinfo.input_components;
    JSAMPROW row_pointer[1];
    while (cinfo.next_scanline < cinfo.image_height)
    {
      row_pointer[0] = (JSAMPROW)&raster[cinfo.next_scanline * row_stride];
      jpeg_write_scanlines(&cinfo, row_pointer, 1);
    }
    jpeg_finish_compress(&cinfo);
    ros_jpeg_mutex_unlock();
    uint32_t compressed_size = (uint32_t)(compress_buf_alloc_size - 
                                          cinfo.dest->free_in_buffer);
    return compressed_size;
  }
  const uint8_t *get_compress_buf() { return compress_buf; }
protected:
  uint8_t *raster, *compress_buf;
  int raster_width, raster_height;
  int raster_alloc_size, compress_buf_alloc_size;
  jpeg_compress_struct cinfo;
  jpeg_decompress_struct dinfo;
  jpeg_error_mgr jerr;

  static void buffer_dest_init(j_compress_ptr cinfo) { }
  static void buffer_source_init(j_decompress_ptr cinfo) { }
  static boolean buffer_source_fill(j_decompress_ptr dinfo)
  {
    if (dinfo->src->bytes_in_buffer == 0)
    {
      printf("woah! jpeg memory buffer was incomplete! ahhhh\n");
      return FALSE;
    }
    return TRUE;
  }
  static void buffer_source_skip(j_decompress_ptr dinfo, long num_bytes)
  {
    dinfo->src->next_input_byte += (size_t)num_bytes;
    dinfo->src->bytes_in_buffer -= (size_t)num_bytes;
  }
  static void buffer_source_term(j_decompress_ptr dinfo) { }
  static boolean buffer_dest_empty(j_compress_ptr cinfo)
  {
    // TODO: enlarge the compression buffer by a factor of 2 and
    // copy over everything, then reset the write pointer
    // and the number of available bytes
    JpegWrapper::g_wrapper->grow_buffer_dest(cinfo->dest->next_output_byte,
                                             cinfo->dest->free_in_buffer);
//    cinfo->dest->next_output_byte = (JOCTET *)(+ compress_buf_alloc_size);
//    cinfo->dest->free_in_buffer = compress_buf_alloc_size;
    return TRUE;
  }
  void grow_buffer_dest(JOCTET *&next_output_byte, size_t &free_in_buffer)
  {
    printf("grow buffer dest\n");
    uint8_t *bigger_dest = new uint8_t[compress_buf_alloc_size * 2];
    memcpy(bigger_dest, compress_buf, compress_buf_alloc_size);
    delete[] compress_buf;
    next_output_byte = (JOCTET *)(bigger_dest + compress_buf_alloc_size);
    compress_buf = bigger_dest;
    free_in_buffer = compress_buf_alloc_size;
    compress_buf_alloc_size *= 2;
  }
  static void buffer_dest_term(j_compress_ptr cinfo) { }
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
  static void jpeg_buffer_dest(j_compress_ptr cinfo, char *buf, int size)
  {
    if (cinfo->dest == NULL)
      cinfo->dest = (struct jpeg_destination_mgr *)(*cinfo->mem->alloc_small)
                    ((j_common_ptr)cinfo, JPOOL_PERMANENT,
                    sizeof(struct jpeg_destination_mgr));
    cinfo->dest->init_destination = buffer_dest_init;
    cinfo->dest->empty_output_buffer = buffer_dest_empty;
    cinfo->dest->term_destination = buffer_dest_term;
    cinfo->dest->next_output_byte = (JOCTET *)buf;
    cinfo->dest->free_in_buffer = size;
  }
  void realloc_raster_if_needed()
  {
    if (raster_alloc_size < raster_size())
    {
      if (raster)
        delete[] raster;
      raster_alloc_size = raster_size();
      if (raster_alloc_size)
        raster = new uint8_t[raster_alloc_size];
      else
        raster = NULL;
    }
  }
};


#endif

