#include <cstdio>
#include "borg.h"

using namespace borg;

int main(int, char **)
{
  Borg borg(Borg::INIT_CAM);
  borg.scan();
  return 0;
}
