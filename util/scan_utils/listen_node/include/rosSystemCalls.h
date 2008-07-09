#ifndef _rossystemcalls_h_
#define _rossystemcalls_h_

enum RosState{ROS_NEW,ROS_READY,ROS_DONE};

void initROS();
void shutdownROS();
RosState getROSState();

#endif
