#ifndef UVC_CAM_H
#define UVC_CAM_H

#include <string>
#include <linux/videodev2.h>

namespace uvc_cam
{

class Cam
{
public:
  Cam(const char *device);
  ~Cam();
  static void enumerate();
  int grab(unsigned char **frame);
  void release(int buf_idx);
private:
  std::string device;
  int fd;
  v4l2_format fmt;
  v4l2_capability cap;
  v4l2_streamparm streamparm;
  v4l2_requestbuffers rb;
  v4l2_buffer buf;
  v4l2_timecode timecode;
  static const unsigned NUM_BUFFER = 4;
  void *mem[NUM_BUFFER];
  unsigned buf_length;
};

}

#endif

