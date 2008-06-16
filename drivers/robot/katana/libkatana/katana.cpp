#include <cstdio>
#include <stdexcept>
#include <vector>
#include "kniBase.h"
#include "katana/katana.h"
#include "ros/common.h"

using std::string;
using std::vector;

Katana::Katana()
{
  // this constructor will throw exceptions if it doesn't work
  // snarf in the text file containing the serial port string
  string katana_pkg_path = ros::get_package_path("katana");
  if (!katana_pkg_path.length())
    throw std::runtime_error("couldn't see package 'katana' using rospack");
  string serial_port_file = katana_pkg_path + "/cfg/serial_port";
  FILE *f = fopen(serial_port_file.c_str(), "r");
  if (!f)
  {
    // write this file if it doesn't already exist
    f = fopen(serial_port_file.c_str(), "w");
    if (!f)
      throw std::runtime_error("couldn't create serial port config file");
    fprintf(f, "/dev/ttyUSB0"); // guess
    fclose(f);
    f = fopen(serial_port_file.c_str(), "r");
    if (!f)
      throw std::runtime_error("couldn't re-open serial port config file");
  }
  char serial_port_cstr[200] = "/dev/ttyS0"; // something reasonable for guess
  fscanf(f, "%200s", serial_port_cstr);
  fclose(f);
  TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, 300, 0, string(serial_port_cstr)};
  device = new CCdlCOM(ccd);
  printf("opened serial port [%s] successfully\n", serial_port_cstr);
  protocol = new CCplSerialCRC();
  protocol->init(device);
  printf("initialized KNI protocol successfully\n");
  kni_lm = new CLMBase();
  kni_lm->create((ros::get_package_path("katana") +
                  "/cfg/stair1_katana.cfg").c_str(), protocol);
  printf("KNI lm base library is up\n");
}

Katana::~Katana()
{
  delete kni_lm;
  delete protocol;
  delete device;
}

bool Katana::calibrate()
{
  kni_lm->calibrate();
  return true;
}

bool Katana::set_motor_power(bool on)
{
  if (on)
    kni_lm->switchRobotOn();
  else
    kni_lm->switchRobotOff();
  return true;
}

bool Katana::goto_joint_position_deg(double j1, double j2, double j3,
                                     double j4, double j5)
{
  vector<double> joints;
  joints.push_back(j1);
  joints.push_back(j2);
  joints.push_back(j3);
  joints.push_back(j4);
  joints.push_back(j5);
  kni_lm->moveRobotToDeg(joints.begin(), joints.end(), true);
  // todo: catch exceptions and be smart
  return true;
}

bool Katana::goto_joint_position_deg(int idx, double pos)
{
  kni_lm->movDegrees(idx, pos, true);
  return true;
}

bool Katana::gripper_fullstop(bool open_gripper)
{
  try
  {
    if (open_gripper)
      kni_lm->openGripper(true, 5000); // todo: think about a reasonable timeout
    else
      kni_lm->closeGripper(true, 5000);
    return true;
  }
  catch(Exception &e)
  {
    return false;
  }
}

vector<double> Katana::get_joint_positions()
{
  return kni_lm->getJointAngles();
}

vector<int> Katana::get_joint_encoders()
{
  return kni_lm->getRobotEncoders();
}

