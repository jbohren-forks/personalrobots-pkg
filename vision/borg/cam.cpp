#include <cstdio>
#include <cstdlib>
#include "cam.h"

using namespace borg;

bool Cam::init()
{
  if (config_status != CAM_NOINIT)
    return false;
  config_status = _init() ? CAM_OK : CAM_ERROR;
  return config_status == CAM_OK;
}

bool Cam::startImageStream()
{
  if (config_status != CAM_OK)
    return false;
  config_status = CAM_STREAMING;
  return _startImageStream();
}

bool Cam::stopImageStream()
{
  if (config_status != CAM_STREAMING)
    return false;
  if (!_stopImageStream())
  {
    config_status = CAM_ERROR;
    return false;
  }
  config_status = CAM_OK;
  return true;
}

uint8_t *Cam::savePhoto()
{
  if (config_status != CAM_STREAMING)
    return NULL;
  return _savePhoto();
}

bool Cam::shutdown()
{
  if (config_status == CAM_OK || config_status == CAM_ERROR)
    return _shutdown();
  return false;
}

