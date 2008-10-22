#ifndef IROBOT_CREATE_H
#define IROBOT_CREATE_H

#include "rosconsole/rosconsole.h"
#include "serial_port/lightweightserial.h"

class IRobotCreate
{
public:
  IRobotCreate();
  ~IRobotCreate();

  bool getEncoders(double &x, double &y, double &th);
  bool setVelocity(double x_vel, double w_vel); // linear, angular velocities

  LightweightSerial *serial_port;
};

#endif

