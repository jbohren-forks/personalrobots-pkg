#include <unistd.h>
#include <cstdio>
#include <cassert>
#include <ros/time.h>
#include "uvc_cam/uvc_cam.h"
#include <signal.h>
#include <stdexcept>
extern "C"
{
#include "avilib.h"
}

const unsigned WIDTH = 640, HEIGHT = 480, FPS = 30;
static bool done = false;

void sigint_handler(int sig)
{
  done = true;
}

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    fprintf(stderr, "usage: view DEVICE\n");
    return 1;
  }
  avi_t avi;
  ros::Time t_prev(ros::Time::now());
  char fname[200];
  sprintf(fname, "%.3f.avi", t_prev.to_double());
  if (AVI_open_output_file(&avi, fname) < 0)
    throw std::runtime_error("couldn't open AVI file");
  AVI_set_video(&avi, WIDTH, HEIGHT, FPS, "MJPG");

  uvc_cam::Cam cam(argv[1], uvc_cam::Cam::MODE_MJPG);
  int count = 0, keyframe = 1;
  signal(SIGINT, sigint_handler);
  while (!done)
  {
    unsigned char *frame = NULL;
    uint32_t bytes_used;
    int buf_idx = cam.grab(&frame, bytes_used);
    printf("      %d byte image\n", bytes_used);
    if (count++ % 30 == 0)
    {
      ros::Time t(ros::Time::now());
      ros::Duration d(t - t_prev);
      printf("%.1f fps\n", 30.0 / d.to_double());
      t_prev = t;
    }
    if (frame)
    {
      AVI_write_frame(&avi, frame, bytes_used, keyframe);
      //fwrite(frame, bytes_used, 1, f);
      cam.release(buf_idx);
      if (keyframe) keyframe = 0;
      /*
      memcpy(surf->pixels, frame, WIDTH*HEIGHT*3);
      cam.release(buf_idx);
      */
    }
  }
  AVI_close(&avi);
  printf("goodbye\n");
  return 0;
}

