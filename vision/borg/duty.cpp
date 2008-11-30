#include <cstdlib>
#include <cstdio>
#include "borg.h"

using namespace borg;

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("usage: duty DUTY GOAL\n");
    return 0;
  }
  Borg borg(Borg::INIT_STAGE);
  borg.stage->setDuty(atoi(argv[1]));
  borg.stage->gotoPosition(atof(argv[2]), false);
  usleep(1000000);
  borg.stage->setDuty(0);
  return 0;
}
