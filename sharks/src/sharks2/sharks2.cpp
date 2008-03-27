// You must have ncurses installed on your system. 
// on Ubuntu: sudo apt-get install libncurses-dev
#include <ncurses.h>
#include "ros/ros_slave.h"
#include "ros/ros_flowpair.h"
#include "common_flows/FlowInt32.h"
#include "common_flows/FlowFloat64.h"
#include "common_flows/FlowEmpty.h"
#include "image_flows/FlowImage.h"
#include "image_flows/image_flow_codec.h"
#include "ipdcmot/FlowPatrol.h"

class Sharks : public ROS_Slave
{
public:
  FlowEmpty *shutter;
  FlowFloat64 *setpos_request, *getpos_result;
  FlowEmpty *getpos_request;
  FlowInt32 *setpos_result;
  FlowImage *image;
  ImageFlowCodec<FlowImage> *codec;

  double start_pos, end_pos, step;
  ROS_FlowPair *setpos_pair, *getpos_pair, *cam_pair;
  bool setpos_ok, image_ok;
  double last_pos;
  int image_count;

  Sharks() : ROS_Slave(), image_count(0)
  {
    register_source(shutter = new FlowEmpty("shutter"));
    register_source(setpos_request = new FlowFloat64("setpos_request"));
    register_source(getpos_request = new FlowEmpty("getpos_request"));
    register_sink(image = new FlowImage("image"), ROS_CALLBACK(Sharks, image_callback));
    register_sink(setpos_result = new FlowInt32("setpos_result"), ROS_CALLBACK(Sharks, setpos_result_callback));
    register_sink(getpos_result = new FlowFloat64("getpos_result"), ROS_CALLBACK(Sharks, getpos_result_callback));
    setpos_pair = new ROS_FlowPair(setpos_request, setpos_result);
    getpos_pair = new ROS_FlowPair(getpos_request, getpos_result);
    cam_pair = new ROS_FlowPair(shutter, image);
    register_with_master();
    codec = new ImageFlowCodec<FlowImage>(image);
    start_pos = -10;
    end_pos = 10;
    step = 1;
    get_double_param(".start", &start_pos);
    get_double_param(".end", &end_pos);
    get_double_param(".step", &step);
  }
  virtual ~Sharks() { }

  bool move_motor(double target, double *actual)
  {
    setpos_request->data = target;
    setpos_pair->publish_and_block(30.0);
    if (!setpos_ok)
    {
      printf("setpos not OK\n");
      return false;
    }
    if (actual)
    {
      getpos_pair->publish_and_block(5.0);
      *actual = last_pos;
    }
    return true;
  }

  void scan()
  {
    if (step == 0)
      step = 1;
    if (start_pos < end_pos && step < 0)
      step *= -1;
    else if (start_pos > end_pos && step > 0)
      step *= -1;
    for (double pos = start_pos; pos < end_pos; pos += step)
    {
      double actual;
      if (!move_motor(pos, &actual))
      {
        printf("error moving to %f\n", pos);
        break;
      }
      mvprintw(1,0,"commanded %f actual %f\n", pos, actual);
      image_ok = false;
      cam_pair->publish_and_block(5.0);
      if (image_ok)
      {
        char fnamebuf[500];
        image_count++;
        snprintf(fnamebuf, sizeof(fnamebuf), 
          "shark_%05d_%010.5f_.jpg", image_count, actual);
        codec->write_file(string(fnamebuf), 95);
      }
    }
  }
  void setpos_result_callback()
  {
    setpos_ok = (setpos_result->data == 1);
  }
  void getpos_result_callback()
  {
    last_pos = getpos_result->data;
  }
  void image_callback()
  {
    mvprintw(2,0,"received a %d by %d image\n", image->width, image->height);
    image_ok = true;
  }
};

void restore_terminal()
{
  echo();
  noraw();
  endwin();
}

int main(int argc, char **argv)
{
  Sharks s;
  atexit(restore_terminal);
  initscr();
  noecho();
  nodelay(stdscr, FALSE);
  mvprintw(0, 0, "Press s to scan\n");
  while (s.happy())
  {
    raw();
    int c = getch();
    noraw();
    if (c == 0x03 || c == 0x04)
      break;
    else if (c == 's')
      s.scan();
  }
  printf("sharks exiting\n");
  return 0;
}

