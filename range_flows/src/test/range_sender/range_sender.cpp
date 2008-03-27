#include <sstream>
#include "ros/ros_slave.h"
#include "range_flows/FlowRangeScan.h"

class RangeSender : public ROS_Slave
{
public:
  FlowRangeScan *my_scan;
  double freq;

  RangeSender() : ROS_Slave(), freq(1)
  {
    register_source(my_scan = new FlowRangeScan("scan"));
    my_scan->set_scan_size(10);
    printf("TALKER params:\n");
    print_param_names();
    printf("EO talker params\n");
    bool b = get_double_param(".freq", &freq);
    printf("b = %d freq = %f\n", b, freq);
  }
  virtual ~RangeSender() { }
  void send_scan()
  {
    for (int i = 0; i < my_scan->get_scan_size(); i++)
      my_scan->scan[i] = rand() % 100;
    my_scan->publish();
  }
};

int main(int argc, char **argv)
{
  RangeSender rs;
  printf("entering talker spin loop\n");
  int sleep_usecs = (int)(1000000.0 / rs.freq);
  printf("sleep usecs = %d\n", sleep_usecs);
  while (rs.happy())
  {
    usleep(sleep_usecs);
    rs.send_scan();
  }
  printf("rangesender says GOODNIGHT\n");
  return 0;
}
