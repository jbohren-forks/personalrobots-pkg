#ifndef IMAGE_UTILS_JPEG_WRAPPER_H
#define IMAGE_UTILS_JPEG_WRAPPER_H

#include "ijg_libjpeg/ros_jpeg_mutex.h"
extern "C"
{
#include "jpeglib.h"
}

class JpegWrapper
{
public:
  JpegWrapper() 
  : raster_width(0), raster_height(0), raster(NULL),
    raster_alloc_size(0)
  {
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
protected:
  uint8_t *raster;
  int raster_width, raster_height;
  int raster_alloc_size;
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
    // TODO
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

