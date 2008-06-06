#include <cstdio>
#include "kniBase.h"
//#include "KNI/kmlMotBase.h"
#include "katana/katana.h"

using std::string;

Katana::Katana(const string &serial_port_str)
{
  // this constructor will throw exceptions if it doesn't work
  TCdlCOMDesc ccd = {0, 57600, 8, 'N', 1, 300, 0, serial_port_str};
  device = new CCdlCOM(ccd);
  printf("opened serial port [%s] successfully\n", serial_port_str.c_str());
  protocol = new CCplSerialCRC();
  protocol->init(device);
  printf("initialized KNI protocol successfully\n");
  kni_lm = new CLMBase();
  kni_lm->create("../../cfg/stair1_katana.cfg", protocol);
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
  return true;
}

