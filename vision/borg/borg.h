#ifndef BORG_H
#define BORG_H

#include "ros/types.h"
#include "cam.h"

namespace borg
{

class Borg
{
public:
  const static uint32_t INIT_CAM   = 0x1;
  const static uint32_t INIT_LASER = 0x2;

  Borg(uint32_t opts);
  ~Borg();
  
  Cam *cam;
private:
  int fps;
};

}

#endif

