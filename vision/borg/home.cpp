#include <cstdio>
#include "borg.h"

using namespace borg;

int main(int, char **)
{
  Borg borg(Borg::INIT_STAGE);
  borg.stage->home();
  return 0;
}
