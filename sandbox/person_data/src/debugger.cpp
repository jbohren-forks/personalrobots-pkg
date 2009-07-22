#include <ros/ros.h>
#include <std_msgs/String.h>
void chatterCallback(const std_msgs::StringConstPtr & msg)
{
	ROS_INFO("Received [%s]",msg->data.c_str());
}
int main(int argc, char ** argv)
{
	ros::init(argc, argv, "debugger");
	ros::NodeHandle n;
	ros::Subscriber chatter_sub = n.subscribe("ia3n_debug",100,chatterCallback);
	ros::spin();
}
