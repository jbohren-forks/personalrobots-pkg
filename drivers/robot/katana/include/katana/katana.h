#ifndef KATANA_KATANA_H
#define KATANA_KATANA_H

#include <string>
#include "kniBase.h"

class Katana
{
public:
  Katana(const std::string &serial_port_str);
  ~Katana();

  bool calibrate();
private:
  CCdlCOM *device;
  CCplSerialCRC *protocol;
  CLMBase *kni_lm;
};

#endif

