#include <stdexcept>
#include <cassert>
#include <cstdio>
#include "SDL/SDL.h"
#include "ros/time.h"
#include "borg.h"
#include <vector>

using namespace borg;
using std::vector;

int main(int argc, char **argv)
{
  if (argc == 1)
    throw std::runtime_error("usage: project EXTRACTIONFILE");
  Borg borg(0);
  vector<Borg::SensedPoint> extraction;
  borg.loadExtractionFile(argv[1], extraction);
  vector<Borg::ProjectedPoint> projection;
  borg.project(extraction, projection);
  for (vector<Borg::ProjectedPoint>::iterator p = projection.begin();
       p != projection.end(); ++p)
    printf("%.3f %.3f %.3f 0 0 0 %.3f %.3f %.3f 0\n", 
           p->x, p->y, p->z,
           p->r / 255.0, p->g / 255.0, p->b / 255.0);
  return 0;
}

