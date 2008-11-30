#ifndef BORG_H
#define BORG_H

#include "ros/types.h"
#include "cam.h"
#include "stage.h"

namespace borg
{

class Borg
{
public:
  const static uint32_t INIT_CAM   = 0x1;
  const static uint32_t INIT_STAGE = 0x2;

  Borg(uint32_t opts);
  ~Borg();
  
  Cam *cam;
  Stage *stage;
  bool scan();
private:
  int fps;
  double left, right;
  int scan_duty, return_duty;
};

}

#endif

