#include "cam.h"

using namespace borg;

bool Cam::init()
{
  if (config_status != CAM_NOINIT)
    return false;
  config_status = _init() ? CAM_OK : CAM_ERROR;
  return config_status == CAM_OK;
}

bool Cam::takePhoto(ImageSize size, uint8_t *raster)
{
  if (config_status == CAM_SHUTDOWN)
    return false;
  else if (config_status == CAM_NOINIT)
  {
    if (!_init())
      return false;
  }
  if (config_status == CAM_OK)
    return _takePhoto(size, raster);
  else
    return false;
}

bool Cam::shutdown()
{
  if (config_status == CAM_OK || config_status == CAM_ERROR)
    return _shutdown();
  return false;
}

