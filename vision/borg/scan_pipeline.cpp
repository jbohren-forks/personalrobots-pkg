#include <cstdio>
#include <vector>
#include <list>
#include "borg.h"

using namespace borg;
using std::list;
using std::vector;

int main(int, char **)
{
  Borg borg(Borg::INIT_CAM | Borg::INIT_STAGE | Borg::INIT_SILENT);
  list<Borg::Image *> images;
  borg.scan(images);
  borg.extract(images, false);
  vector<Borg::SensedPoint> extraction;
  borg.sensedPoints(images, extraction);
  vector<Borg::ProjectedPoint> projection;
  borg.project(extraction, projection);
  for (vector<Borg::ProjectedPoint>::iterator p = projection.begin();
       p != projection.end(); ++p)
    printf("%.3f %.3f %.3f %d %d %d\n", p->x, p->y, p->z, p->r, p->g, p->b);
  return 0;
}

