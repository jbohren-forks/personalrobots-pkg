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
  // todo: add higher-resolution modes if we ever use them
  // right now, our algorithms can barely handle 640x480
  // so I don't really have a need for more than that...
  enum video_mode_t
  {
    FLEA2_MONO,
    FLEA2_RGB
  } video_mode;

  Flea2(video_mode_t _video_mode = FLEA2_MONO, int n_host = 0);
  ~Flea2();

  JpegWrapper jpeg;
  raw1394handle_t raw;
  dc1394_cameracapture cam;
  nodeid_t cam_node;
  bool raw_created, cam_created;
  bool frame_released;

  // this grabs a frame, jpegs it, and releases the frame
  bool get_jpeg(const uint8_t ** const fetch_jpeg_buf, uint32_t *fetch_buf_size);

  bool get_frame(uint8_t ** const frame, uint32_t *width, uint32_t *height); 
  // for mow, you *must* call release_frame after you're done with the frame
  void release_frame();

  void set_shutter(double d); // range = [0,1]
  void set_gamma(double g); // range = [0,1]
  void set_gain(double g); // range = [0,1]
};

#endif

