#include <cstdlib>
#include <cstdio>
#include "borg.h"

using namespace borg;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: goto POSITION\n");
    return 0;
  }
  Borg borg(Borg::INIT_STAGE);
  borg.stage->gotoPosition(atof(argv[1]), true);
  return 0;
}
