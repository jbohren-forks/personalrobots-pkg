#include "listen_node/rosSystemCalls.h"

#include <ros/node.h>
#include <stdio.h>

RosState state = ROS_NEW;

void initROS()
{
	if (state != ROS_NEW) {
		fprintf(stderr,"Wrong ROS state in initROS\n");
		return;
	}
			   
	int argc = 0;
	ros::init(argc,NULL);
	fprintf(stderr,"ROS started\n");
	state = ROS_READY;
}

void shutdownROS()
{
	if (state != ROS_READY) {
		fprintf(stderr,"Wrong ROS state in shutdownROS\n");
		return;
	}
			   
	ros::fini();
	fprintf(stderr,"ROS finished\n");
	state = ROS_DONE;
}

RosState getROSState()
{
	return state;
}
