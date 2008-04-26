#include <math.h>
#include "ros/ros_slave.h"
#include "range_flows/FlowRangeScan.h"
#include "hokuyo_top_urg/hokuyo_top_urg.h"

class TopUrgNode : public ROS_Slave
{
public:
  FlowRangeScan *scan;
  string urg_dev;
  HokuyoTopUrg *h;

  TopUrgNode() : ROS_Slave()
  {
    register_source(scan = new FlowRangeScan("scan"));
    scan->start_angle = -HokuyoTopUrg::SCAN_FOV / 2.0;
    scan->angle_increment = HokuyoTopUrg::SCAN_FOV / (double)HokuyoTopUrg::num_ranges;
    if (!get_string_param(".device", urg_dev))
    {
      printf("woah! param 'device' not specified in the graph. defaulting to /dev/null\n");
      urg_dev = "/dev/null";
    }
    printf("URG device set to [%s]\n", urg_dev.c_str());
    h = new HokuyoTopUrg(urg_dev);
    scan->set_scan_size(h->num_ranges);
  }
  bool send_scan()
  {
    if (!h->poll())
    {
      printf("couldn't poll the laser\n");
      return false;
    }
    for (int i = 0; i < h->num_ranges; i++)
      scan->scan[i] = h->latest_scan[i];
    scan->publish();
    return true;
  }
};

int main(int argc, char **argv)
{
  TopUrgNode t;
  while (t.happy())
    if (!t.send_scan())
    {
      printf("couldn't get/send scan\n");
      return 1;
    }
  printf("unhappy\n");
  return 0;
}
