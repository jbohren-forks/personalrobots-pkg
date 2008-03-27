#include <cstdio>
#include "hokuyo_top_urg/hokuyo_top_urg.h"

int main(int argc, char **argv)
{
  HokuyoTopUrg h("/dev/ttyACM0");
  while(h.is_ok())
  {
    char line[80] = {0};
    fgets(line, 80, stdin);
    printf("press enter to poll a scan. type 'q<enter>' to quit...\n");
    if (line[0] == 'q')
      break;
    h.poll();
    for (int i = 0; i < h.num_ranges; i += 200)
      printf("range %d = %f meters\n", i, h.latest_scan[i]);
  }
  return 0;
}

