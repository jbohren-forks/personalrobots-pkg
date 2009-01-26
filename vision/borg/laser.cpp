#include <cstdlib>
#include <cstdio>
#include "borg.h"

using namespace borg;

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    printf("usage: laser CMD\n"
           "where CMD=1 will turn it on, anything else turns it off.\n");
    return 0;
  }
  Borg borg(Borg::INIT_STAGE);
  borg.stage->laser((argv[1][0] == '1' ? true : false));
  return 0;
}
