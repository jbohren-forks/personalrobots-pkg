#include <cstdio>
#include "cam_dc1394.h"

using namespace borg;

int main(int, char **)
{
  Cam *cam = new CamDC1394();
  cam->init();
  uint8_t *raster = new uint8_t[640*480];
  cam->takePhoto(IMAGESIZE_640_480, raster);
  cam->shutdown();
  delete[] raster;
  return 0;
}
