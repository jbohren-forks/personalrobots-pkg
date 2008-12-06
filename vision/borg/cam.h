#ifndef BORG_CAM_H
#define BORG_CAM_H

#include "borg_types.h"

namespace borg
{

class Cam
{
public:
  Cam() : config_status(CAM_NOINIT) { }
  virtual ~Cam() { }

  bool init();
  bool savePhoto(uint8_t *);
  bool shutdown();
  bool startImageStream();
  bool stopImageStream();
  virtual bool set(const char *setting, uint32_t value) = 0;
  bool writePgm(const char *filename, const uint8_t *raster);

protected:
  virtual bool _init() { return true; }
  virtual bool _savePhoto(uint8_t *) = 0;
  virtual bool _shutdown() { return true; }
  virtual bool _startImageStream() = 0;
  virtual bool _stopImageStream() = 0;
  enum { CAM_NOINIT, CAM_OK, CAM_STREAMING, CAM_ERROR, CAM_SHUTDOWN } config_status;
};

}

#endif

