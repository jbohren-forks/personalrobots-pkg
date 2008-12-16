#ifndef BORG_CAM_DC1394_H
#define BORG_CAM_DC1394_H

#include "cam.h"
#include "dc1394/dc1394.h"

namespace borg
{

class CamDC1394 : public Cam
{
public:
  CamDC1394();
  virtual ~CamDC1394();
  virtual bool set(const char *setting, uint32_t value);
  virtual void prepareStill();
  virtual void prepareScan();
protected:
  virtual bool _init();
  virtual bool _savePhoto(uint8_t *);
  virtual bool _shutdown();
  virtual bool _startImageStream();
  virtual bool _stopImageStream();
private:
  dc1394_t *dc;
  dc1394camera_t *cam;
  uint32_t scan_gain, still_gain, scan_shutter, still_shutter;
};

}

#endif

