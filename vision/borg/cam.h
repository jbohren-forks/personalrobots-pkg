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
  uint8_t *savePhoto();
  bool shutdown();
  bool startImageStream();
  bool stopImageStream();

protected:
  virtual bool _init() { return true; }
  virtual uint8_t *_savePhoto() = 0;
  virtual bool _shutdown() { return true; }
  virtual bool _startImageStream() = 0;
  virtual bool _stopImageStream() = 0;
  enum { CAM_NOINIT, CAM_OK, CAM_STREAMING, CAM_ERROR, CAM_SHUTDOWN } config_status;
};

}

#endif

