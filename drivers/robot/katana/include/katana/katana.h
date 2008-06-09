#ifndef KATANA_KATANA_H
#define KATANA_KATANA_H

#include <string>
#include <vector>
#include "kniBase.h"

class Katana
{
public:
  Katana();
  ~Katana();

  bool calibrate();
  bool set_motor_power(bool on);
  std::vector<double> get_joint_positions();
  std::vector<int>    get_joint_encoders();
  bool goto_joint_position_deg(double j1, double j2, double j3, 
                               double j4, double j5);
  bool goto_joint_position_deg(int motor_idx, double motor_position);
  bool gripper_fullstop(bool open_gripper); // for binary gripper control

private:
  CCdlCOM *device;
  CCplSerialCRC *protocol;
  CLMBase *kni_lm;
};

#endif

