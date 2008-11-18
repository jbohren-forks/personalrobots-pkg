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
protected:
  virtual bool _init();
  virtual bool _takePhoto(ImageSize size, uint8_t *raster);
  virtual bool _shutdown();
private:
  dc1394_t *dc;
  dc1394camera_t *cam;
};

}

#endif

