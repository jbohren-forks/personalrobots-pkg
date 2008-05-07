#ifndef FLEA2
#define FLEA2

#include <cstdio>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <cstdlib>
#include "image_utils/jpeg_wrapper.h"

class Flea2
{
public:
  Flea2(int n_host = 0);
  ~Flea2();

  JpegWrapper jpeg;
  raw1394handle_t raw;
  dc1394_cameracapture cam;
  bool raw_created, cam_created;
  bool frame_released;

  // this grabs a frame, jpegs it, and releases the frame
  bool get_jpeg(const uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size);

  bool get_frame(uint8_t ** const frame, uint32_t *width, uint32_t *height); 
  // for mow, you *must* call release_frame after you're done with the frame
  void release_frame();
};

#endif

