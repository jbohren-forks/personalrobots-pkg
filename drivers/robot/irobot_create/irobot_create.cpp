#include "rosconsole/rosconsole.h"
#include "irobot_create/irobot_create.h"

IRobotCreate::IRobotCreate()
{
  // see if the IROBOT_CREATE_PORT variable is defined
  const char *default_port = "/dev/ttyUSB0";
  const char *port_env = getenv("IROBOT_CREATE_PORT");
  const char *serial_port_str = (port_env ? port_env : default_port);
  ROS_DEBUG("irobotcreate ctor, serial_port_str = [%s]", serial_port_str);
  serial_port = new LightweightSerial(serial_port_str, 115200);
}

IRobotCreate::~IRobotCreate()
{
  ROS_DEBUG("irobotcreate dtor");
  ROS_DEBUG("closing serial port");
  delete serial_port;
  serial_port = NULL;
  ROS_DEBUG("irobotcreate dtor complete");
}

bool IRobotCreate::getEncoders(double &x, double &y, double &th)
{
  x = 0;
  y = 0; 
  th = 0;
  return true;
}

bool IRobotCreate::setVelocity(double x_vel, double w_vel)
{
  ROS_DEBUG("setting velocity to %f, %f", x_vel, w_vel);
  return true;
}

