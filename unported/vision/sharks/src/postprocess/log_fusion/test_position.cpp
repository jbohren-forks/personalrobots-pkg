#include <cstdio>
#include "ipdcmot/ipdcmot.h"

int main(int argc, char **argv)
{
  IPDCMOT *mot = new IPDCMOT("192.168.1.38", 75);
  printf("sleeping...\n");
  usleep(1000000);
  printf("going to 20 degrees\n");
  mot->set_pos_deg_blocking(20);
  printf("done. going to 40 degrees\n");
  mot->set_pos_deg_blocking(40);
  printf("done.\n");


  delete mot;
  return 0;
}

