#include <cstdio>
#include <list>
#include "borg.h"

using namespace borg;
using std::list;

int main(int, char **)
{
  Borg borg(Borg::INIT_CAM | Borg::INIT_STAGE);
  list<Borg::Image *> images;
  borg.scan(images);
  borg.saveScan(images, "out/img_");
  return 0;
}

