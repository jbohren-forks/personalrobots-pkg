#ifndef BORG_CAM_H
#define BORG_CAM_H

#include "borg_types.h"

namespace borg
{

class Cam
{
public:
  Cam() { }
  virtual ~Cam() { }

  bool init();
  bool takePhoto(ImageSize size, uint8_t *raster);
  bool shutdown();

protected:
  virtual bool _init() { return true; }
  virtual bool _takePhoto(ImageSize size, uint8_t *raster) = 0;
  virtual bool _shutdown() { return true; }
  enum { CAM_NOINIT, CAM_OK, CAM_ERROR, CAM_SHUTDOWN } config_status;
};

}

#endif

