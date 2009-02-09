#include <cassert>
#include <cstdio>
#include "SDL/SDL.h"
#include "ros/time.h"
#include "borg.h"

using namespace borg;

int main(int, char **)
{
  printf("borg ctor\n");
  Borg borg(Borg::INIT_CAM);
  printf("borg startimagestream\n");
  borg.cam->startImageStream();
  printf("borg preparestill\n");
  borg.cam->prepareStill();
  Borg::Image *image = new Borg::Image(new uint8_t[640*480],
                                       ros::Time::now().toSec(), 0);
  for (int flush = 0; flush < 5; flush++)
    if (!borg.cam->savePhoto(image->raster))
      break;
  borg.cam->stopImageStream();
  borg.cam->writePgm("grabbed.pgm", image->raster);
  return 0;
}

