#include <stdexcept>
#include <cassert>
#include <cstdio>
#include "SDL/SDL.h"
#include "ros/time.h"
#include "borg.h"
#include <list>
#include <string>

using namespace borg;
using std::list;
using std::string;

int main(int argc, char **argv)
{
  if (argc < 5)
    throw std::runtime_error("usage: project SQ_SIZE X Y SCENEPREFIX1 ...");
  Borg borg(0);
  std::list<string> prefixes;
  for (int i = 4; i < argc; i++)
    prefixes.push_back(argv[i]);
  borg.calibrate(atof(argv[1]), atoi(argv[2]), atoi(argv[3]), prefixes);
  return 0;
}

