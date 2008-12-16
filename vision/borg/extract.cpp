#include <cassert>
#include <cstdio>
#include "SDL/SDL.h"
#include "ros/time.h"
#include "borg.h"
#include <list>

using std::list;
using namespace borg;

int main(int argc, char **argv)
{
  Borg borg(0);
  list<Borg::Image *> images;
  for (int a = 1; a < argc; a++)
    images.push_back(new Borg::Image(argv[a]));
  borg.extract(images, false);
  borg.printExtraction(images);
  for (list<Borg::Image *>::iterator i = images.begin(); i != images.end(); ++i)
  {
    delete (*i)->raster;
    delete *i;
  }
  images.clear();
  return 0;
}

