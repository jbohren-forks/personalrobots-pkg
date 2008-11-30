#include <cstdlib>
#include <cstdio>
#include "borg.h"
#include "ros/time.h"

using namespace borg;

int main(int argc, char **argv)
{
  if (argc < 4)
  {
    printf("usage: scan_enc_test LEFT RIGHT DUTY\n");
    return 0;
  }
  const double left = atof(argv[1]), right = atof(argv[2]);
  Borg borg(Borg::INIT_STAGE);
  borg.stage->setDuty(600);
  borg.stage->gotoPosition(left, true);
  borg.stage->setDuty(atoi(argv[3]));
  borg.stage->gotoPosition(right, false);
  ros::Time t_start(ros::Time::now());
  for (ros::Time t_now(ros::Time::now()); (t_now - t_start).to_double() < 15;
       t_now = ros::Time::now())
  {
    double pos = borg.stage->getPosition(1.0);
    if (fabs(pos - right) < 0.5)
      break;
    printf("%.3f %.3f\n", t_now.to_double(), pos);
  }
  return 0;
}
