#ifndef FLEA2
#define FLEA2

#include <cstdio>
#include <libraw1394/raw1394.h>
#include <libdc1394/dc1394_control.h>
#include <cstdlib>
#include <stdint.h>

class FwHost {
public:
  FwHost(int n_host = 0);
  ~FwHost();

  nodeid_t *nodes;
  int num_nodes;
  int num_cams;

  char host_dev[200];
  raw1394handle_t raw;

  bool raw_created;

};


class FwCam
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

  FwCam(FwHost* host, int n_dev = 0, int chan = -1, video_mode_t _video_mode = FLEA2_MONO);
  ~FwCam();

  FwHost* mp_host;

  bool get_frame(uint8_t ** const frame, uint32_t *width, uint32_t *height); 

  void release_frame();

  dc1394_cameracapture cam;

  nodeid_t cam_node;
  bool frame_released;

  bool cam_created;

};

#endif

