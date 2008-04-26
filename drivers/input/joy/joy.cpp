#include <unistd.h>
#include <math.h>
#include <linux/joystick.h>
#include <fcntl.h>
#include "ros/node.h"
#include "joy/MsgJoy.h"
#include "std_msgs/MsgEmpty.h"

void *s_joy_func(void *);
using namespace ros;

class Joy : public node
{
public:
  MsgJoy joy_msg;
  MsgEmpty deadman_msg;

  int joy_fd;
  string joy_dev;
  int joy_buttons;

  Joy() : node("joy")
  {
    param("joy_dev", joy_dev, "/dev/input/js0");
    joy_fd = open(joy_dev.c_str(), O_RDONLY);
    if (joy_fd <= 0)
      log(FATAL, "couldn't open joystick %s.", joy_dev.c_str());
    pthread_t joy_thread;
    pthread_create(&joy_thread, NULL, s_joy_func, this);

    advertise("joy", joy_msg);
    advertise("deadman", deadman_msg);
  }
  void monitor_deadman()
  {
    while (ok())
    {
      usleep(100000);
      if (joy_buttons & 0x20)
        publish("deadman", deadman_msg);
    }
  }
  void joy_func()
  {
    js_event event;
    while (ok())
    {
      read(joy_fd, &event, sizeof(js_event));
      if (event.type & JS_EVENT_INIT)
        continue;
      switch(event.type)
      {
        case JS_EVENT_BUTTON:
          if (event.value)
            joy_buttons |= (1 << event.number);
          else
            joy_buttons &= ~(1 << event.number);
          publish("joy", joy_msg);
          break;
        case JS_EVENT_AXIS:
          switch(event.number)
          {
            case 0: joy_msg.x1 =  event.value / 32767.0; break;
            case 1: joy_msg.y1 = -event.value / 32767.0; break;
            case 2: joy_msg.x2 =  event.value / 32767.0; break;
            case 3: joy_msg.y2 = -event.value / 32767.0; break;
            default: break;
          }
          publish("joy", joy_msg);
          break;
      }
    }
  }
};

void *s_joy_func(void *parent)
{
  ((Joy *)parent)->joy_func();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  Joy joy;
  joy.monitor_deadman();
  return 0;
}

