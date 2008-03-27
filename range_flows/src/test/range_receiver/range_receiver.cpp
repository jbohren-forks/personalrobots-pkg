#include <sstream>
#include "ros/ros_slave.h"
#include "range_flows/FlowRangeScan.h"

class RangeReceiver : public ROS_Slave
{
public:
  FlowRangeScan *my_scan;
  double freq;

  RangeReceiver() : ROS_Slave(), freq(1)
  {
    register_sink(my_scan = new FlowRangeScan("scan"), ROS_CALLBACK(RangeReceiver, range_cb));
    printf("params:\n");
    print_param_names();
    printf("EO talker params\n");
    bool b = get_double_param("freq", &freq);
    printf("b = %d freq = %f\n", b, freq);
  }
  virtual ~RangeReceiver() { }
  void range_cb()
  {
    printf("count = %d secs = %d nsecs = %d\n", my_scan->publish_count, my_scan->get_stamp_secs(), my_scan->get_stamp_nsecs());
    /*
    for (int i = 0; i < my_scan->get_scan_size(); i++)
      printf("range %d = %f\n", i, my_scan->scan[i]);
    */
  }
};

int main(int argc, char **argv)
{
  RangeReceiver rr;
  printf("entering receiver spin loop\n");
  rr.spin();
  return 0;
}
