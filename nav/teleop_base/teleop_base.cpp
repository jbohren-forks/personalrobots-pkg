#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/MsgJoy.h"
#include "std_msgs/MsgBaseVel.h"

using namespace ros;

class TeleopBase : public node
{
public:
  MsgBaseVel cmd;
  MsgJoy joy;
  double req_vx, req_vw, max_vx, max_vw;

  TeleopBase() : node("teleop_base"), max_vx(1), max_vw(1)
  {
    cmd.vx = cmd.vw = 0;
    if (!has_param("max_vx") || !get_param("max_vx", max_vx))
      log(WARNING, "maximum linear velocity (max_vx) not set. Assuming 0.3");
    if (!has_param("max_vw") || !get_param("max_vw", max_vx))
      log(WARNING, "maximum angular velocity (max_vw) not set. Assuming 0.3");
    advertise("cmd_vel", cmd);
    subscribe("joy", joy, &TeleopBase::joy_cb);
  }
  void joy_cb()
  {
    joy.lock();
    req_vx =  joy.y1 * max_vx;
    req_vw = -joy.x2 * max_vw;
    joy.unlock();
  }
  void send_cmd_vel()
  {
    joy.lock();
    if (joy.buttons & 0x20) // todo: pull the "deadman" button from a paramter
    {
      cmd.vx = req_vx;
      cmd.vw = req_vw;
    }
    else
      cmd.vx = cmd.vw = 0;
    joy.unlock();
    publish("cmd_vel", cmd);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv);
  TeleopBase teleop_base;
  while (teleop_base.ok())
  {
    usleep(200000);
    teleop_base.send_cmd_vel();
  }
  return 0;
}

